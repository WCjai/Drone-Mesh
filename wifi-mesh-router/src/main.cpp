// ROOT NO-ROUTER
#include "ardupilotmega/mavlink.h"
#include <string.h>
#include <inttypes.h>
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include "nvs_flash.h"
#include <algorithm> 
#include "driver/uart.h"
#include <vector>

#define DEBUG_MESH 1  // 0 pour désactiver les logs, 1 pour les activer
#define LR_PROTOCOL 0 //1 pour activer le mode longue range
// Remplacer tous les MESH_LOGE par une macro personnalisée
#define MESH_LOGE(...) do { if (DEBUG_MESH) ESP_LOGE(__VA_ARGS__); } while (0)

/* Mesh WIFI config*/
#define CONFIG_MESH_AP_PASSWD "12345678"
#define CONFIG_MESH_ROUTE_TABLE_SIZE 50
static const uint8_t MESH_ID[6] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77};
#define MAX_RETRY 3


#define BUFFER_SIZE 1024
#define MESH_PACKET_SIZE 280
int baudrate = 921600;
const uart_port_t CONFIG_UART_PORT_NUM = UART_NUM_1;
int Rtos_delay = 10;//90;

// Error counter to track persistent issues
static int error_count_tx = 0;
static int error_count_rx = 0;
static const int ERROR_THRESHOLD = 10;

static uint8_t tx_buf[BUFFER_SIZE] = { 0 };
static uint8_t rx_buf[MESH_PACKET_SIZE] = { 0 };

static bool is_mesh_connected = false;
static mesh_addr_t mesh_parent_addr;
static int mesh_layer = -1;
static esp_netif_t *netif_sta = NULL;
static bool is_running = true;
mesh_addr_t my_address;
int InSerialPacketSize = 0;
int InTcpPacketSize = 0;

// Structure pour stocker les informations des nœuds enfants
struct ChildNodeInfo {
    mesh_addr_t addr;
    uint8_t layer;
    int64_t last_seen;
    bool is_direct_child;
};

class MeshChildManager {
private:
    std::vector<ChildNodeInfo> children;
    static const uint8_t CHILD_INFO_MSG_TYPE = 0x01;
    
    struct ChildInfoPacket {
        uint8_t msg_type;
        uint8_t num_children;
        uint8_t layer;
        mesh_addr_t child_addresses[6];  // Maximum 6 children per node
    };

public:
    // Ajouter ou mettre à jour un nœud enfant
    void updateChild(const mesh_addr_t& addr, uint8_t layer, bool is_direct) {
        auto it = std::find_if(children.begin(), children.end(),
            [&addr](const ChildNodeInfo& child) {
                return memcmp(child.addr.addr, addr.addr, 6) == 0;
            });

        if (it != children.end()) {
            it->last_seen = esp_timer_get_time();
            it->layer = layer;
            it->is_direct_child = is_direct;
        } else {
            ChildNodeInfo new_child = {
                .addr = addr,
                .layer = layer,
                .last_seen = esp_timer_get_time(),
                .is_direct_child = is_direct
            };
            children.push_back(new_child);
        }
    }

    // Supprimer un nœud enfant
    void removeChild(const mesh_addr_t& addr) {
        children.erase(
            std::remove_if(children.begin(), children.end(),
                [&addr](const ChildNodeInfo& child) {
                    return memcmp(child.addr.addr, addr.addr, 6) == 0;
                }),
            children.end());
    }

    // Envoyer les informations des nœuds enfants à tous les autres nœuds
    esp_err_t broadcastChildrenInfo() {
        if (children.empty()) {
            return ESP_OK;
        }

        ChildInfoPacket packet = {
            .msg_type = CHILD_INFO_MSG_TYPE,
            .num_children = static_cast<uint8_t>(std::min(children.size(), size_t(6))),
            .layer = static_cast<uint8_t>(esp_mesh_get_layer())
        };

        // Copier les adresses des enfants dans le paquet
        for (size_t i = 0; i < packet.num_children && i < 6; i++) {
            memcpy(&packet.child_addresses[i], &children[i].addr, sizeof(mesh_addr_t));
        }

        // Préparer les données mesh
        mesh_data_t data = {
            .data = reinterpret_cast<uint8_t*>(&packet),
            .size = sizeof(ChildInfoPacket),
            .proto = MESH_PROTO_BIN,
            .tos = MESH_TOS_P2P
        };

        // Obtenir la table de routage
        mesh_addr_t route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
        int route_table_size = 0;
        esp_mesh_get_routing_table(route_table, CONFIG_MESH_ROUTE_TABLE_SIZE * 6, &route_table_size);

        // Envoyer à tous les nœuds sauf soi-même
        for (int i = 0; i < route_table_size; i++) {
            if (memcmp(route_table[i].addr, my_address.addr, 6) != 0) {
                esp_err_t err = esp_mesh_send(&route_table[i], &data, MESH_DATA_P2P, NULL, 0);
                if (err != ESP_OK) {
                    MESH_LOGE("MESH", "Failed to send child info to node %d: %s", 
                            i, esp_err_to_name(err));
                }
            }
        }

        return ESP_OK;
    }

    // Traiter les informations reçues des autres nœuds
    void processReceivedChildInfo(const uint8_t* data, size_t size) {
        if (size < sizeof(ChildInfoPacket)) {
            return;
        }

        const ChildInfoPacket* packet = reinterpret_cast<const ChildInfoPacket*>(data);
        if (packet->msg_type != CHILD_INFO_MSG_TYPE) {
            return;
        }

        for (uint8_t i = 0; i < packet->num_children; i++) {
            updateChild(packet->child_addresses[i], packet->layer + 1, false);
        }
    }
};

// Instance globale du gestionnaire
MeshChildManager childManager;

void setup();
void mesh_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data);
void ip_event_handler(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data);

void loop();


void esp_mesh_p2p_tx_main(void *arg)
{
    mesh_addr_t route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
    int route_table_size = 0;
    is_running = true;

    while (is_running) {
        // Get the routing table only once or when necessary to reduce overhead
        esp_mesh_get_routing_table(route_table, CONFIG_MESH_ROUTE_TABLE_SIZE * 6, &route_table_size);

        size_t available_bytes = 0;
        uart_get_buffered_data_len(CONFIG_UART_PORT_NUM, &available_bytes);

        if (available_bytes > 0) {
            int len = uart_read_bytes(CONFIG_UART_PORT_NUM, tx_buf, sizeof(tx_buf), pdMS_TO_TICKS(Rtos_delay));
            
            if (len > 0) { 
                mavlink_message_t msg;
                mavlink_status_t status;
                for (int i = 0; i < len; ++i) {
                    if (mavlink_parse_char(MAVLINK_COMM_0, tx_buf[i], &msg, &status)) {
                        uint8_t send_buf[MAVLINK_MAX_PACKET_LEN];
                        int send_len = mavlink_msg_to_send_buffer(send_buf, &msg);

                        // Batch send with optimizations
                        int offset = 0;
                        while (offset < send_len) {
                            int packetSize = (send_len - offset) > MESH_PACKET_SIZE ? MESH_PACKET_SIZE : (send_len - offset);
                            uint8_t message[MESH_PACKET_SIZE];
                            memcpy(message, send_buf + offset, packetSize);
                            
                            mesh_data_t data;
                            data.data = message;
                            data.size = packetSize;
                            data.proto = MESH_PROTO_BIN;
                            data.tos = MESH_TOS_P2P;

                            // Iterate over the route table and send data
                            for (int i = 0; i < route_table_size; i++) {
                                if (memcmp(route_table[i].addr, my_address.addr, 6) != 0) {
                                    esp_err_t err = esp_mesh_send(&route_table[i], &data, MESH_DATA_P2P, NULL, 0);
                                    if (err != ESP_OK) {
                                        error_count_tx++;
                                        err = esp_mesh_send(&route_table[i], &data, MESH_DATA_P2P, NULL, 0);
                                        if (err == ESP_OK) {
                                            error_count_tx = 0; // Reset on success
                                        } else {
                                            MESH_LOGE("MESH", "Retry failed, error: %d", err);
                                        }
                                    } else {
                                        error_count_tx = 0; // Reset error count on success
                                    }

                                    // Stop task if too many errors
                                    if (error_count_tx >= ERROR_THRESHOLD) {
                                        MESH_LOGE("MESH", "Too many transmission errors, stopping task");
                                        is_running = false;
                                        break;
                                    }
                                }
                            }
                            offset += packetSize;
                        }
                    }
                }
            } else if (len < 0) {
                MESH_LOGE("UART", "Error reading from UART: %d", len);
                error_count_tx++;
            }
        }

        vTaskDelay(1); // Yield to other tasks
    }

    vTaskDelete(NULL);
}

void esp_mesh_p2p_rx_main(void *arg)
{
    mesh_addr_t from;
    mesh_data_t data;
    int flag = 0;
    is_running = true;

    static uint8_t reassemblyBuffer[BUFFER_SIZE];
    static int receivedLength = 0;

    while (is_running) {
        data.data = rx_buf;
        data.size = sizeof(rx_buf);
        
        esp_err_t err = esp_mesh_recv(&from, &data, portMAX_DELAY, &flag, NULL, 0);
        if (err != ESP_OK) {
            MESH_LOGE("MESH", "Error receiving data: %d", err);
            error_count_rx++;
            if (error_count_rx >= ERROR_THRESHOLD) {
                MESH_LOGE("MESH", "Too many reception errors, stopping task");
                is_running = false;
            }
            continue;
        }

        error_count_rx = 0; // Reset on successful reception

        if (receivedLength + data.size <= BUFFER_SIZE) {
            memcpy(reassemblyBuffer + receivedLength, data.data, data.size);
            receivedLength += data.size;

            mavlink_message_t message;
            mavlink_status_t status;
            int bytesParsed = 0;

            for (int i = 0; i < receivedLength; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, reassemblyBuffer[i], &message, &status)) {
                    uint8_t send_buf[MAVLINK_MAX_PACKET_LEN];
                    int send_len = mavlink_msg_to_send_buffer(send_buf, &message);
                    uart_write_bytes(CONFIG_UART_PORT_NUM, send_buf, send_len);   
                    bytesParsed = i + 1;
                }
            }

            // Shift remaining bytes in the buffer
            if (bytesParsed > 0) {
                memmove(reassemblyBuffer, reassemblyBuffer + bytesParsed, receivedLength - bytesParsed);
                receivedLength -= bytesParsed;
            }
        } else {
            MESH_LOGE("MESH", "Buffer overflow detected. Message too large.");
            receivedLength = 0;
        }

        vTaskDelay(1); // Yield to other tasks
    }

    vTaskDelete(NULL);
}



esp_err_t esp_mesh_comm_p2p_start(void)
{
    static bool is_comm_p2p_started = false;
    if (!is_comm_p2p_started) {
        is_comm_p2p_started = true;
        xTaskCreate(esp_mesh_p2p_tx_main, "MPTX", 8000, NULL, 5, NULL);
        xTaskCreate(esp_mesh_p2p_rx_main, "MPRX", 8000, NULL, 5, NULL);
    }
    return ESP_OK;
}


void ip_event_handler(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    MESH_LOGE("MESH", "<IP_EVENT_STA_GOT_IP>IP:" IPSTR, IP2STR(&event->ip_info.ip));
}


void mesh_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data)
{
    mesh_addr_t id = {0,};
    static uint16_t last_layer = 0;

    switch (event_id) {
    case MESH_EVENT_STARTED: {
        esp_mesh_get_id(&id);
        MESH_LOGE("MESH", "<MESH_EVENT_MESH_STARTED>ID:" MACSTR "", MAC2STR(id.addr));
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_STOPPED: {
        MESH_LOGE("MESH", "<MESH_EVENT_STOPPED>");
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_CHILD_CONNECTED: {
        mesh_event_child_connected_t *child_connected = (mesh_event_child_connected_t *)event_data;
        MESH_LOGE("MESH", "<MESH_EVENT_CHILD_CONNECTED>aid:%d, " MACSTR "",
                 child_connected->aid,
                 MAC2STR(child_connected->mac));
        esp_mesh_comm_p2p_start();
        mesh_addr_t child_addr;
        memcpy(child_addr.addr, child_connected->mac, 6);
        childManager.updateChild(child_addr, esp_mesh_get_layer() + 1, true);
        childManager.broadcastChildrenInfo();
    }
    break;
    case MESH_EVENT_CHILD_DISCONNECTED: {
        mesh_event_child_disconnected_t *child_disconnected = (mesh_event_child_disconnected_t *)event_data;
        MESH_LOGE("MESH", "<MESH_EVENT_CHILD_DISCONNECTED>aid:%d, " MACSTR "",
                 child_disconnected->aid,
                 MAC2STR(child_disconnected->mac));
        mesh_addr_t child_addr;
        memcpy(child_addr.addr, child_disconnected->mac, 6);
        childManager.removeChild(child_addr);
        childManager.broadcastChildrenInfo();
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_ADD: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        MESH_LOGE("MESH", "<MESH_EVENT_ROUTING_TABLE_ADD>add %d, new:%d, layer:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new, mesh_layer);
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_REMOVE: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        MESH_LOGE("MESH", "<MESH_EVENT_ROUTING_TABLE_REMOVE>remove %d, new:%d, layer:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new, mesh_layer);
    }
    break;
    case MESH_EVENT_NO_PARENT_FOUND: {
        mesh_event_no_parent_found_t *no_parent = (mesh_event_no_parent_found_t *)event_data;
        MESH_LOGE("MESH", "<MESH_EVENT_NO_PARENT_FOUND>scan times:%d",
                 no_parent->scan_times);
    }
    /* TODO handler for the failure */
    break;
    case MESH_EVENT_PARENT_CONNECTED: {
        mesh_event_connected_t *connected = (mesh_event_connected_t *)event_data;
        esp_mesh_get_id(&id);
        mesh_layer = connected->self_layer;
        memcpy(&mesh_parent_addr.addr, connected->connected.bssid, 6);
        MESH_LOGE("MESH",
                 "<MESH_EVENT_PARENT_CONNECTED>layer:%d-->%d, parent:" MACSTR "%s, ID:" MACSTR ", duty:%d",
                 last_layer, mesh_layer, MAC2STR(mesh_parent_addr.addr),
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "", MAC2STR(id.addr), connected->duty);
        last_layer = mesh_layer;
        is_mesh_connected = true;
        if (esp_mesh_is_root()) {
            esp_netif_dhcpc_stop(netif_sta);
        }
        esp_mesh_comm_p2p_start();
    }
    break;
    case MESH_EVENT_PARENT_DISCONNECTED: {
        mesh_event_disconnected_t *disconnected = (mesh_event_disconnected_t *)event_data;
        MESH_LOGE("MESH",
                 "<MESH_EVENT_PARENT_DISCONNECTED>reason:%d",
                 disconnected->reason);
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
        
    }
    break;
    case MESH_EVENT_LAYER_CHANGE: {
        mesh_event_layer_change_t *layer_change = (mesh_event_layer_change_t *)event_data;
        mesh_layer = layer_change->new_layer;
        MESH_LOGE("MESH", "<MESH_EVENT_LAYER_CHANGE>layer:%d-->%d%s",
                 last_layer, mesh_layer,
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "");
        last_layer = mesh_layer;
       
    }
    break;
    case MESH_EVENT_ROOT_ADDRESS: {
        mesh_event_root_address_t *root_addr = (mesh_event_root_address_t *)event_data;
        MESH_LOGE("MESH", "<MESH_EVENT_ROOT_ADDRESS>root address:" MACSTR "",
                 MAC2STR(root_addr->addr));
    }
    break;
    case MESH_EVENT_VOTE_STARTED: {
        mesh_event_vote_started_t *vote_started = (mesh_event_vote_started_t *)event_data;
        MESH_LOGE("MESH",
                 "<MESH_EVENT_VOTE_STARTED>attempts:%d, reason:%d, rc_addr:" MACSTR "",
                 vote_started->attempts,
                 vote_started->reason,
                 MAC2STR(vote_started->rc_addr.addr));
    }
    break;
    case MESH_EVENT_VOTE_STOPPED: {
        MESH_LOGE("MESH", "<MESH_EVENT_VOTE_STOPPED>");
        break;
    }
    case MESH_EVENT_ROOT_SWITCH_REQ: {
        mesh_event_root_switch_req_t *switch_req = (mesh_event_root_switch_req_t *)event_data;
        MESH_LOGE("MESH",
                 "<MESH_EVENT_ROOT_SWITCH_REQ>reason:%d, rc_addr:" MACSTR "",
                 switch_req->reason,
                 MAC2STR( switch_req->rc_addr.addr));
    }
    break;
    case MESH_EVENT_ROOT_SWITCH_ACK: {
        /* new root */
        mesh_layer = esp_mesh_get_layer();
        esp_mesh_get_parent_bssid(&mesh_parent_addr);
        MESH_LOGE("MESH", "<MESH_EVENT_ROOT_SWITCH_ACK>layer:%d, parent:" MACSTR "", mesh_layer, MAC2STR(mesh_parent_addr.addr));
    }
    break;
    case MESH_EVENT_TODS_STATE: {
        mesh_event_toDS_state_t *toDs_state = (mesh_event_toDS_state_t *)event_data;
        MESH_LOGE("MESH", "<MESH_EVENT_TODS_REACHABLE>state:%d", *toDs_state);
    }
    break;
    case MESH_EVENT_ROOT_FIXED: {
        mesh_event_root_fixed_t *root_fixed = (mesh_event_root_fixed_t *)event_data;
        MESH_LOGE("MESH", "<MESH_EVENT_ROOT_FIXED>%s",
                 root_fixed->is_fixed ? "fixed" : "not fixed");
    }
    break;
    case MESH_EVENT_ROOT_ASKED_YIELD: {
        mesh_event_root_conflict_t *root_conflict = (mesh_event_root_conflict_t *)event_data;
        MESH_LOGE("MESH",
                 "<MESH_EVENT_ROOT_ASKED_YIELD>" MACSTR ", rssi:%d, capacity:%d",
                 MAC2STR(root_conflict->addr),
                 root_conflict->rssi,
                 root_conflict->capacity);
    }
    break;
    case MESH_EVENT_CHANNEL_SWITCH: {
        mesh_event_channel_switch_t *channel_switch = (mesh_event_channel_switch_t *)event_data;
        MESH_LOGE("MESH", "<MESH_EVENT_CHANNEL_SWITCH>new channel:%d", channel_switch->channel);
    }
    break;
    case MESH_EVENT_SCAN_DONE: {
        mesh_event_scan_done_t *scan_done = (mesh_event_scan_done_t *)event_data;
        MESH_LOGE("MESH", "<MESH_EVENT_SCAN_DONE>number:%d",
                 scan_done->number);
    }
    break;
    case MESH_EVENT_NETWORK_STATE: {
        mesh_event_network_state_t *network_state = (mesh_event_network_state_t *)event_data;
        MESH_LOGE("MESH", "<MESH_EVENT_NETWORK_STATE>is_rootless:%d",
                 network_state->is_rootless);
    }
    break;
    case MESH_EVENT_STOP_RECONNECTION: {
        MESH_LOGE("MESH", "<MESH_EVENT_STOP_RECONNECTION>");
    }
    break;
    case MESH_EVENT_FIND_NETWORK: {
        mesh_event_find_network_t *find_network = (mesh_event_find_network_t *)event_data;
        MESH_LOGE("MESH", "<MESH_EVENT_FIND_NETWORK>new channel:%d, router BSSID:" MACSTR "",
                 find_network->channel, MAC2STR(find_network->router_bssid));
    }
    break;
    case MESH_EVENT_ROUTER_SWITCH: {
        mesh_event_router_switch_t *router_switch = (mesh_event_router_switch_t *)event_data;
        MESH_LOGE("MESH", "<MESH_EVENT_ROUTER_SWITCH>new router:%s, channel:%d, " MACSTR "",
                 router_switch->ssid, router_switch->channel, MAC2STR(router_switch->bssid));
    }
    break;
    case MESH_EVENT_PS_PARENT_DUTY: {
        mesh_event_ps_duty_t *ps_duty = (mesh_event_ps_duty_t *)event_data;
        MESH_LOGE("MESH", "<MESH_EVENT_PS_PARENT_DUTY>duty:%d", ps_duty->duty);
    }
    break;
    case MESH_EVENT_PS_CHILD_DUTY: {
        mesh_event_ps_duty_t *ps_duty = (mesh_event_ps_duty_t *)event_data;
        MESH_LOGE("MESH", "<MESH_EVENT_PS_CHILD_DUTY>cidx:%d, " MACSTR ", duty:%d", ps_duty->child_connected.aid-1,
                MAC2STR(ps_duty->child_connected.mac), ps_duty->duty);
    }
    break;
    default:
        MESH_LOGE("MESH", "unknown id:%" PRId32 "", event_id);
        break;
    }
}


void setup() {

    uart_config_t uart_config = {
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(CONFIG_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(CONFIG_UART_PORT_NUM, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_UART_PORT_NUM, 2 * BUFFER_SIZE, 2 * BUFFER_SIZE, 0, NULL, 0));


    ESP_ERROR_CHECK(nvs_flash_init());
    /*  tcpip initialization */
    ESP_ERROR_CHECK(esp_netif_init());
    /*  event initialization */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    /*  create network interfaces for mesh (only station instance saved for further manipulation, soft AP instance ignored */
    ESP_ERROR_CHECK(esp_netif_create_default_wifi_mesh_netifs(&netif_sta, NULL));
    /*  wifi initialization */
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_start());
    if (LR_PROTOCOL){
    const uint8_t protocol = WIFI_PROTOCOL_LR;//longue distance
    ESP_ERROR_CHECK( esp_wifi_set_protocol(WIFI_IF_AP, protocol) );// longue distance
    }
    /* Set the maximum Wi-Fi TX power */
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(80));  // Set TX power to 20.5 dBm (maximum)
    /*  gets this esp mac address, to prevent sending message to itself */
    esp_wifi_get_mac(WIFI_IF_STA, my_address.addr);

    /*  mesh initialization */
    ESP_ERROR_CHECK(esp_mesh_init());
    ESP_ERROR_CHECK(esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler, NULL));
    /*  set mesh topology */
    ESP_ERROR_CHECK(esp_mesh_set_topology(MESH_TOPO_TREE));
    /*  set mesh max layer according to the topology */
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(6));
    ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1));
    ESP_ERROR_CHECK(esp_mesh_set_xon_qsize(128));

    /* Disable mesh PS function */
    ESP_ERROR_CHECK(esp_mesh_disable_ps());
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(120));

    /* set this esp as root node*/ 
    ESP_ERROR_CHECK(esp_mesh_set_type(MESH_ROOT));
   
    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
    /* mesh ID */
    memcpy((uint8_t *) &cfg.mesh_id, MESH_ID, 6);
    /* router */
    cfg.channel = 0;

    /* mesh softAP */
    ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(WIFI_AUTH_WPA_WPA2_PSK));
    cfg.mesh_ap.max_connection = 6;
    cfg.mesh_ap.nonmesh_max_connection = 0;
    memcpy((uint8_t *) &cfg.mesh_ap.password, CONFIG_MESH_AP_PASSWD,
           strlen(CONFIG_MESH_AP_PASSWD));
    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));

    /* mesh start */
    ESP_ERROR_CHECK(esp_mesh_start());
    MESH_LOGE("MESH", "mesh starts successfully, heap:%" PRId32 ", %s<%d>%s, ps:%d",  esp_get_minimum_free_heap_size(),
             esp_mesh_is_root_fixed() ? "root fixed" : "root not fixed",
             esp_mesh_get_topology(), esp_mesh_get_topology() ? "(chain)":"(tree)", esp_mesh_is_ps_enabled());
    
}

void loop()
{
}
