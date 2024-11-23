// NODE NO-ROUTER

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
#include <WiFi.h>
#include "driver/uart.h"
#include <map>
#include <vector>
#include <math.h>

#define DEBUG_MESH 1  // 0 pour désactiver les logs, 1 pour les activer
#define LR_PROTOCOL 0 // 1 pour activer le wifi en mode longue range
// Remplacer tous les MESH_LOGE par une macro personnalisée
#define MESH_LOGE(...) do { if (DEBUG_MESH) ESP_LOGE(__VA_ARGS__); } while (0)

// Structure pour les informations des nœuds
struct MeshNodeInfo {
    mesh_addr_t addr;
    int rssi;
    int layer;
    bool is_root;
    bool is_parent;  // Ajout du flag parent
    bool is_child;
    bool is_self;
    uint32_t last_seen;
    int child_count;
    int8_t tx_power; 
};


// Classe pour gérer la liste des adresses
class MeshAddressList {
private:
    std::vector<MeshNodeInfo> nodes;
    static const uint32_t NODE_TIMEOUT = 60000; // 30 secondes timeout
    mesh_addr_t current_root_addr;
    mesh_addr_t current_parent_addr;  // Stockage de l'adresse du parent
    mesh_addr_t self_addr;  // Adresse du nœud courant
    int current_layer;      // Couche actuelle du nœud
public:
     
     
     MeshAddressList() {
        memset(&current_root_addr, 0, sizeof(mesh_addr_t));
        memset(&current_parent_addr, 0, sizeof(mesh_addr_t));
        memset(&self_addr, 0, sizeof(mesh_addr_t));
    } 
    
   void adjustTxPower(MeshNodeInfo& node, int rssi) {
        const int RSSI_TARGET_MIN = -65;  // RSSI cible minimum acceptable
        const int RSSI_TARGET_MAX = -55;  // RSSI cible maximum acceptable
        const int8_t TX_POWER_MIN = 8;    // 8 dBm minimum
        const int8_t TX_POWER_MAX = 84;   // 20 dBm maximum (84/4 = 21dBm)
        
        // Si le RSSI est trop faible, augmenter la puissance
        if (rssi < RSSI_TARGET_MIN && node.tx_power < TX_POWER_MAX) {
            node.tx_power = std::min<int8_t>(node.tx_power + 4, TX_POWER_MAX);
            esp_wifi_set_max_tx_power(node.tx_power);
            MESH_LOGE("MESH", "Increasing TX power to %d for node " MACSTR ", RSSI: %d", 
                     node.tx_power, MAC2STR(node.addr.addr), rssi);
        }
        // Si le RSSI est trop fort, diminuer la puissance
        else if (rssi > RSSI_TARGET_MAX && node.tx_power > TX_POWER_MIN) {
            node.tx_power = std::max<int8_t>(node.tx_power - 4, TX_POWER_MIN);
            esp_wifi_set_max_tx_power(node.tx_power);
            MESH_LOGE("MESH", "Decreasing TX power to %d for node " MACSTR ", RSSI: %d", 
                     node.tx_power, MAC2STR(node.addr.addr), rssi);
        }
    }

    void updateSelfNode() {
        // Get MAC address using ESP-MESH API
        esp_err_t ret = esp_mesh_get_id(&self_addr);
        MESH_LOGE("MESH", "1 Update Self node " MACSTR "",MAC2STR(self_addr.addr));
        
        if ((ret != ESP_OK) || !isValidMacAddress(self_addr.addr)){
            // Fallback: try to get MAC from WiFi station interface
            uint8_t temp_mac[6];
            ret = esp_wifi_get_mac(WIFI_IF_STA, temp_mac);
            if ((ret == ESP_OK) && isValidMacAddress(self_addr.addr)){
                memcpy(self_addr.addr, temp_mac, 6);
                MESH_LOGE("MESH", "2 Update Self node " MACSTR "",MAC2STR(self_addr.addr));
            } else {
                // Last resort: try getting base MAC address
                ret = esp_efuse_mac_get_default(self_addr.addr);
                MESH_LOGE("MESH", "3 Update Self node " MACSTR "",MAC2STR(self_addr.addr));
                if (ret != ESP_OK) {
                    MESH_LOGE("MESH", "No Self node MAC");
                }
            }
        }
    }


    bool isValidMacAddress(const uint8_t* addr) {
        // Vérifie si l'adresse est 00:00:00:00:00:00
        int u=0;
        int v=0;
        for (int i = 0; i < 6; i++) {
            if (addr[i] == 0x00) {
                u++;
            }else if (addr[i] == 0x77) {
                v++;
            }
        }
        if ((u==6)  ||(v==6))
        return false;
        else 
        return true;

    }
    MeshNodeInfo getRootAddr() {
        MeshNodeInfo addr;
        bool ok=false;
        
        for (const auto& node : nodes) {
            if (node.is_root){
                ok=true;
                addr=node;
                break;
            }
        }
        if (ok){
            return addr;
        }
        
    }

    MeshNodeInfo getParentAddr() {
        MeshNodeInfo addr;
        bool ok=false;
        
        for (const auto& node : nodes) {
            if (node.is_parent){
                ok=true;
                addr=node;
                break;
            }
        }
        if (ok){
            return addr;
        }
        
    }
 
    // Mettre à jour l'adresse du parent
    void updateParentAddress(const mesh_addr_t& parent_addr) {
        memcpy(&current_parent_addr.addr, parent_addr.addr, 6);
    }

    // Obtenir l'adresse du parent actuel
    mesh_addr_t getParentAddress() {
        return current_parent_addr;
    }

    std::vector<mesh_addr_t> getlistAddr() {
        std::vector<mesh_addr_t> addrs;
        for (const auto& node : nodes) {
            addrs.push_back(node.addr);
        }
        return addrs;
    }

    std::vector<MeshNodeInfo> getlistNodes() {
        
        return nodes;
    }
    // Modifier la fonction getNodeMesh dans la classe MeshAddressList
    std::vector<mesh_addr_t> getNodeMesh() {
        std::vector<mesh_addr_t> addrs;
        
        // Obtenir l'adresse du nœud courant
        mesh_addr_t current_node;
        esp_mesh_get_id(&current_node);
        
        // Ajouter l'adresse du nœud courant si elle est valide
        if (isValidMacAddress(current_node.addr)) {
            addrs.push_back(current_node);
            MESH_LOGE("MESH", "Current node MAC: " MACSTR, MAC2STR(current_node.addr));
        }

        // Récupérer la table de routage
        int route_table_size = esp_mesh_get_routing_table_size();
        if (route_table_size > 0) {
            mesh_addr_t* route_table = (mesh_addr_t*)malloc(route_table_size * sizeof(mesh_addr_t));
            if (route_table != nullptr) {
                esp_err_t err = esp_mesh_get_routing_table(
                    route_table,
                    route_table_size * sizeof(mesh_addr_t),
                    &route_table_size
                );
                
                if (err == ESP_OK) {
                    for (int i = 0; i < route_table_size; i++) {
                        if (isValidMacAddress(route_table[i].addr)) {
                            // Vérifier si l'adresse n'est pas déjà dans la liste
                            bool alreadyExists = false;
                            for (const auto& addr : addrs) {
                                if (memcmp(addr.addr, route_table[i].addr, 6) == 0) {
                                    alreadyExists = true;
                                    break;
                                }
                            }
                            
                            if (!alreadyExists) {
                                addrs.push_back(route_table[i]);
                                MESH_LOGE("MESH", "Found valid node MAC: " MACSTR, MAC2STR(route_table[i].addr));
                            }
                        }
                    }
                }
                free(route_table);
            }
        }
        return addrs;
    }

    // Modifier la fonction NotNodeInTheList pour inclure la validation de l'adresse
    bool NotNodeInTheList(mesh_addr_t addrN) {
        if (!isValidMacAddress(addrN.addr)) {
            return false;  // Ignorer les adresses invalides
        }

        for (const auto& node : nodes) {
            if (memcmp(node.addr.addr, addrN.addr, 6) == 0) {
                return false;
            }
        }
        return true;
    }

 
    // création de la liste des noeuds
    void updateNode(const mesh_addr_t& addr, int layer, bool is_root = false, bool is_self = false,bool is_child = false, int rssi = 0) {
        bool is_parent = memcmp(addr.addr, current_parent_addr.addr, 6) == 0;
        bool found = false;
        bool is_self1 = memcmp(addr.addr, self_addr.addr, 6) == 0;
        
        if (isValidMacAddress(addr.addr)){       
            for (auto& node : nodes) {
                if (memcmp(node.addr.addr, addr.addr, 6) == 0) {
                    node.layer = layer;
                    node.is_root = is_root;
                    node.is_parent = is_parent;
                    node.is_self = is_self1;
                    node.is_child = is_child;
                    node.last_seen = millis();
                    found = true;
                    node.rssi = rssi;  // Mise à jour du RSSI
                    
                    // Ajuster la puissance d'émission si nécessaire
                    if (!is_self1) {
                        adjustTxPower(node, rssi);
                    }     

                    MESH_LOGE("MESH", "Updated node " MACSTR " (Layer: %d%s%s%s%S)", 
                        MAC2STR(addr.addr), 
                        layer,
                        is_root ? ", ROOT" : "",
                        is_parent ? ", PARENT" : "",
                        is_child ? ", CHILD" : "",
                        is_self ? ", SELF" : "");
                    break;
                }
            }    
 
            if (!found) {
                        MeshNodeInfo new_node = {
                            .addr = addr,
                            .rssi = rssi,
                            .layer = layer,
                            .is_root = is_root,
                            .is_parent = is_parent,
                            .is_child = is_child,
                            .is_self = is_self1,
                            .last_seen = millis(),
                            .tx_power = 84  // Démarrer à la puissance maximale
                        };
                                       
                        nodes.push_back(new_node);
                        
                        MESH_LOGE("MESH", "New node added: " MACSTR " (Layer: %d%s%s%s%S)", 
                                MAC2STR(addr.addr), 
                                layer,
                                is_root ? ", ROOT" : "",
                                is_parent ? ", PARENT" : "",
                                is_child ? ", CHILD" : "",
                                is_self ? ", SELF" : "");
            }
        }
            
    }

    // Obtenir notre adresse
    mesh_addr_t getSelfAddress() {
        return self_addr;
    }

    // Obtenir notre couche actuelle
    int getSelfLayer() {
        return current_layer;
    }

    // Vérifier si une adresse est la nôtre
    bool isSelfAddress(const mesh_addr_t& addr) {
        return memcmp(addr.addr, self_addr.addr, 6) == 0;
    }

    // met ajour la derniere fois qu'on la vu
    void NodeSeen(const mesh_addr_t& addr) {
        for (auto& node : nodes) {
            if (memcmp(node.addr.addr, addr.addr, 6) == 0) 
                node.last_seen = millis();
        }
    }

    // Obtenir toutes les adresses actives
    std::vector<mesh_addr_t> getActiveAddresses() {
        cleanupStaleNodes();
        std::vector<mesh_addr_t> addresses;
        for (const auto& node : nodes) {
            addresses.push_back(node.addr);
        }
        return addresses;
    }
    
    // Supprimer un nœud
    void removeNode(const mesh_addr_t& addr) {
        nodes.erase(
            std::remove_if(
                nodes.begin(), 
                nodes.end(),
                [&addr](const MeshNodeInfo& node) {
                    return memcmp(node.addr.addr, addr.addr, 6) == 0;
                }
            ),
            nodes.end()
        );
    }

     // Obtenir le nombre de nœuds
    size_t getNodeCount() {
        cleanupStaleNodes();
        return nodes.size();
    }
 
                 
   // Méthode mise à jour pour l'affichage
    void printNodes() {
       // cleanupStaleNodes();
        MESH_LOGE("MESH", "Current nodes in network (%d) self_address : " MACSTR "", nodes.size(),MAC2STR(self_addr.addr));

        // Afficher d'abord le nœud racine
        for (const auto& node : nodes) {
            if (node.is_root) {
                printNodeInfo(node);
                break;
            }
        }

        // Puis le parent s'il n'est pas la racine
        if (!nodes.empty()) {
            for (const auto& node : nodes) {
                if (node.is_parent && !node.is_root) {
                    printNodeInfo(node);
                    break;
                }
            }
        }

        // Puis tous les autres nœuds
        for (const auto& node : nodes) {
            if (!node.is_root && !node.is_parent) {
                printNodeInfo(node);
            }
        }
    }

    void cleanupOnParentDisconnect() {
        // Mettre à jour notre couche
        current_layer = esp_mesh_get_layer();
        
        // Réinitialiser l'adresse du parent
        memset(&current_parent_addr, 0, sizeof(mesh_addr_t));

        // Mettre à jour notre nœud
        updateSelfNode();

        nodes.erase(
            std::remove_if(
                nodes.begin(),
                nodes.end(),
                [this](const MeshNodeInfo& node) {
                    return !node.is_self && node.layer < current_layer;
                }
            ),
            nodes.end()
        );

        // Mettre à jour les nœuds restants
        for (auto& node : nodes) {
            if (!node.is_self) {
                node.is_parent = false;
                if (node.layer > current_layer) {
                    int layer_diff = node.layer - current_layer;
                    node.layer = current_layer + layer_diff;
                }
            }
        }

        MESH_LOGE("MESH", "Network topology cleaned up after parent disconnect");
        printNodes();
    }
private:
   // Nettoyer les nœuds inactifs
    void cleanupStaleNodes() {
        uint32_t current_time = millis();
        for (auto& node : nodes) {
            if (memcmp(node.addr.addr, self_addr.addr, 6) != 0) {
                nodes.erase(
                    std::remove_if(
                        nodes.begin(),
                        nodes.end(),
                        [current_time](const MeshNodeInfo& node) {
                            return (current_time - node.last_seen) > NODE_TIMEOUT;
                        }
                    ),
                    nodes.end()
                );
            }
        }
    }

     void printNodeInfo(const MeshNodeInfo& node) {
             MESH_LOGE("MESH", "Node " MACSTR " (Layer: %d, RSSI: %d, TX Power: %d%s%s%s%s)", 
                 MAC2STR(node.addr.addr),
                 node.layer,
                 node.rssi,
                 node.tx_power,
                 node.is_root ? ", ROOT" : "",
                 node.is_parent ? ", PARENT" : "",
                 node.is_child ? ", CHILD" : "",
                 node.is_self ? ", SELF" : "");
    }
};


// Instance globale
static MeshAddressList mesh_addresses;

class MeshChildManager {
private:
   
    static const uint8_t CHILD_INFO_MSG_TYPE = 0x01;
    
    struct ChildInfoPacket {
        uint8_t msg_type;
        uint8_t num_children;
        uint8_t layer;
        mesh_addr_t child_addresses[6];  // Maximum 6 children per node
    };

public:

    // Traiter les informations reçues des autres nœuds
    void processReceivedChildInfo(const uint8_t* data, size_t size, int rssi) {
        if (size < sizeof(ChildInfoPacket)) {
            return;
        }

        const ChildInfoPacket* packet = reinterpret_cast<const ChildInfoPacket*>(data);
        if (packet->msg_type != CHILD_INFO_MSG_TYPE) {
            return;
        }

        for (uint8_t i = 0; i < packet->num_children; i++) {
            mesh_addresses.updateNode(packet->child_addresses[i], packet->layer + 1,false,false,true,rssi);
            MESH_LOGE("MESH" , " ProcessReceivedChildinfo " MACSTR " " , MAC2STR(packet->child_addresses[i].addr));
        }
    }
};

MeshChildManager childManager;

/* mesh WIFI config*/
#define CONFIG_MESH_AP_PASSWD "12345678"   //MESH PASSWORD
#define CONFIG_MESH_ROUTE_TABLE_SIZE 50
static const uint8_t MESH_ID[6] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77}; //MESH ID

// Earth's radius in meters
#define EARTH_RADIUS 6371000.0

/* need adjustment*/
#define BUFFER_SIZE 1024
#define MESH_PACKET_SIZE 280

// Buffer sizes
#define MAX_MAVLINK_MSG_SIZE   280

// Structure pour suivre les temps de connexion
struct ConnectionTiming {
    uint32_t start_time;           // Temps de démarrage
    uint32_t first_connection;     // Temps de première connexion
    uint32_t last_disconnection;   // Temps de dernière déconnexion
    uint32_t last_connection;      // Temps de dernière connexion
    uint32_t connection_count;     // Nombre de connexions
    uint32_t min_reconnect_time;   // Temps minimum de reconnexion
    uint32_t max_reconnect_time;   // Temps maximum de reconnexion
    uint32_t total_reconnect_time; // Temps total de reconnexion
    uint32_t reconnection_count;   // Nombre de reconnexions
    bool is_connected;             // État actuel
};


// Structure globale pour tous les types de connexions
// Initialiser les nouvelles variables dans MeshTiming lors de la création
struct MeshTiming {
    ConnectionTiming root;
    ConnectionTiming parent;
    ConnectionTiming child;
    uint32_t mesh_start_time;
} mesh_timing = {
    .root = {0, 0, 0, 0, 0, UINT32_MAX, 0, 0, 0, false},
    .parent = {0, 0, 0, 0, 0, UINT32_MAX, 0, 0, 0, false},
    .child = {0, 0, 0, 0, 0, UINT32_MAX, 0, 0, 0, false},
    .mesh_start_time = 0
};

const uart_port_t CONFIG_UART_PORT_NUM = UART_NUM_2;
int Rtos_delay = 10;//90 c'est pour 115200

#define RX_BUFFER_SIZE MESH_PACKET_SIZE
static uint8_t tx_buf[BUFFER_SIZE] = { 0 };
static uint8_t rx_buf[RX_BUFFER_SIZE] = { 0 };

// Error counter to track persistent issues
static int error_count = 0;
static const int ERROR_THRESHOLD = 10;
mavlink_global_position_int_t myPos;
int mysysid;
static bool is_mesh_connected = false;
static mesh_addr_t mesh_parent_addr;
static int mesh_layer = -1;
static esp_netif_t *netif_sta = NULL;
static bool is_running = true;
mesh_addr_t root_address;
mesh_addr_t Selfaddr;

void setup();
void mesh_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data);
void ip_event_handler(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data);

void loop();


// Fonctions pour gérer les temps
void handleConnection(ConnectionTiming& timing) {
    uint32_t current_time = millis();
    uint32_t time_since_start = current_time - mesh_timing.mesh_start_time;
    
    if (!timing.first_connection) {
        // Première connexion
        timing.first_connection = current_time;
        MESH_LOGE("MESH_TIMING", "First connection: %u ms since start", time_since_start);
    } else if (timing.last_disconnection) {
        // C'est une reconnexion
        uint32_t reconnect_time = current_time - timing.last_disconnection;
        timing.reconnection_count++;
        timing.total_reconnect_time += reconnect_time;
        
        // Mise à jour min/max
        if (reconnect_time < timing.min_reconnect_time || timing.min_reconnect_time == 0) {
            timing.min_reconnect_time = reconnect_time;
        }
        if (reconnect_time > timing.max_reconnect_time) {
            timing.max_reconnect_time = reconnect_time;
        }
        
        MESH_LOGE("MESH_TIMING", "Reconnection after %u ms", reconnect_time);
    }
    
    timing.last_connection = current_time;
    timing.connection_count++;
    timing.is_connected = true;
}

void handleDisconnection(ConnectionTiming& timing) {
    if (timing.is_connected) {
        timing.last_disconnection = millis();
        timing.is_connected = false;
        uint32_t connection_duration = timing.last_disconnection - timing.last_connection;
        MESH_LOGE("MESH_TIMING", "Disconnection after %u ms connection", connection_duration);
    }
}

// Fonction pour afficher un résumé des temps
void printTimingSummary() {
    uint32_t current_time = millis();
    uint32_t uptime = current_time - mesh_timing.mesh_start_time;
    
    MESH_LOGE("MESH_TIMING", "\nMesh Timing Summary:");
    MESH_LOGE("MESH_TIMING", "Uptime: %u ms", uptime);
    
    // Root info
    MESH_LOGE("MESH_TIMING", "\nRoot:");
    MESH_LOGE("MESH_TIMING", "  First connection: %u ms after start", 
             mesh_timing.root.first_connection ? 
             mesh_timing.root.first_connection - mesh_timing.mesh_start_time : 0);
    MESH_LOGE("MESH_TIMING", "  Total connections: %u", mesh_timing.root.connection_count);
    if (mesh_timing.root.reconnection_count > 0) {
        MESH_LOGE("MESH_TIMING", "  Reconnections: %u", mesh_timing.root.reconnection_count);
        MESH_LOGE("MESH_TIMING", "  Average reconnect time: %u ms", 
                 mesh_timing.root.total_reconnect_time / mesh_timing.root.reconnection_count);
        MESH_LOGE("MESH_TIMING", "  Min reconnect time: %u ms", mesh_timing.root.min_reconnect_time);
        MESH_LOGE("MESH_TIMING", "  Max reconnect time: %u ms", mesh_timing.root.max_reconnect_time);
    }
    MESH_LOGE("MESH_TIMING", "  Current status: %s", 
             mesh_timing.root.is_connected ? "Connected" : "Disconnected");
    
    // Parent info
    MESH_LOGE("MESH_TIMING", "\nParent:");
    MESH_LOGE("MESH_TIMING", "  First connection: %u ms after start", 
             mesh_timing.parent.first_connection ? 
             mesh_timing.parent.first_connection - mesh_timing.mesh_start_time : 0);
    MESH_LOGE("MESH_TIMING", "  Total connections: %u", mesh_timing.parent.connection_count);
    if (mesh_timing.parent.reconnection_count > 0) {
        MESH_LOGE("MESH_TIMING", "  Reconnections: %u", mesh_timing.parent.reconnection_count);
        MESH_LOGE("MESH_TIMING", "  Average reconnect time: %u ms", 
                 mesh_timing.parent.total_reconnect_time / mesh_timing.parent.reconnection_count);
        MESH_LOGE("MESH_TIMING", "  Min reconnect time: %u ms", mesh_timing.parent.min_reconnect_time);
        MESH_LOGE("MESH_TIMING", "  Max reconnect time: %u ms", mesh_timing.parent.max_reconnect_time);
    }
    MESH_LOGE("MESH_TIMING", "  Current status: %s", 
             mesh_timing.parent.is_connected ? "Connected" : "Disconnected");
    if (!mesh_timing.parent.is_connected && mesh_timing.parent.last_disconnection) {
        MESH_LOGE("MESH_TIMING", "  Current disconnection duration: %u ms",
                 current_time - mesh_timing.parent.last_disconnection);
    }
    
    // Child info
    MESH_LOGE("MESH_TIMING", "\nChild:");
    MESH_LOGE("MESH_TIMING", "  First connection: %u ms after start", 
             mesh_timing.child.first_connection ? 
             mesh_timing.child.first_connection - mesh_timing.mesh_start_time : 0);
    MESH_LOGE("MESH_TIMING", "  Total connections: %u", mesh_timing.child.connection_count);
    if (mesh_timing.child.reconnection_count > 0) {
        MESH_LOGE("MESH_TIMING", "  Reconnections: %u", mesh_timing.child.reconnection_count);
        MESH_LOGE("MESH_TIMING", "  Average reconnect time: %u ms", 
                 mesh_timing.child.total_reconnect_time / mesh_timing.child.reconnection_count);
        MESH_LOGE("MESH_TIMING", "  Min reconnect time: %u ms", mesh_timing.child.min_reconnect_time);
        MESH_LOGE("MESH_TIMING", "  Max reconnect time: %u ms", mesh_timing.child.max_reconnect_time);
    }
    MESH_LOGE("MESH_TIMING", "  Current status: %s", 
             mesh_timing.child.is_connected ? "Connected" : "Disconnected");
}


double calculate_distance_between_positions(const mavlink_global_position_int_t* pos1, 
                                         const mavlink_global_position_int_t* pos2) {
    // Convert lat/lon from format in GLOBAL_POSITION_INT (degrees * 1E7) to radians
    double lat1 = (pos1->lat / 1E7) * M_PI / 180.0;
    double lon1 = (pos1->lon / 1E7) * M_PI / 180.0;
    double lat2 = (pos2->lat / 1E7) * M_PI / 180.0;
    double lon2 = (pos2->lon / 1E7) * M_PI / 180.0;
    
    // Get altitude difference in meters
    double alt1 = pos1->alt / 1000.0;  // Convert from millimeters to meters
    double alt2 = pos2->alt / 1000.0;
    double alt_diff = alt2 - alt1;

    // Haversine formula
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    
    double a = sin(dlat/2) * sin(dlat/2) +
               cos(lat1) * cos(lat2) * 
               sin(dlon/2) * sin(dlon/2);
    
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    // Surface distance
    double surface_distance = EARTH_RADIUS * c;
    
    // Calculate 3D distance using Pythagorean theorem
    double distance = sqrt(surface_distance * surface_distance + alt_diff * alt_diff);
    
    return distance;
}

 // Convert MAC string to mesh_addr_t a déplacer dans un fichier tools.cpp
static esp_err_t stringToMeshAddr(const char* mac_str, mesh_addr_t& mesh_addr) {
        // Clear the mesh_addr
        memset(&mesh_addr, 0, sizeof(mesh_addr_t));
        
        // Check input string length
        if (strlen(mac_str) != 17) {  // XX:XX:XX:XX:XX:XX = 17 chars
            return ESP_ERR_INVALID_ARG;
        }

        // Convert each hex pair to byte
        uint8_t bytes[6];
        const char* ptr = mac_str;
        
        for (int i = 0; i < 6; i++) {
            char hex[3] = {ptr[0], ptr[1], 0};  // Extract two characters + null terminator
            char* end;
            bytes[i] = strtol(hex, &end, 16);
            
            // Check for conversion errors
            if (end != hex + 2) {
                return ESP_ERR_INVALID_ARG;
            }
            
            ptr += 3;  // Skip two chars and separator
        }

        // Copy to mesh_addr
        memcpy(mesh_addr.addr, bytes, 6);
        return ESP_OK;
}

void esp_mesh_p2p_tx_main(void *arg)
{
    mesh_addr_t route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
    int route_table_size = 0;
    is_running = true;
    uint32_t last_position_broadcast = millis();
    const uint32_t POSITION_BROADCAST_INTERVAL = 10000; // 10 seconds
 
    while (is_running) {
 
        uint32_t current_time = millis();
        // Check if there is data available on the serial port
        size_t available_bytes = 0;
        uart_get_buffered_data_len(CONFIG_UART_PORT_NUM, &available_bytes);
        
        if (available_bytes > 0) {
            int len = uart_read_bytes(CONFIG_UART_PORT_NUM, tx_buf, sizeof(tx_buf), pdMS_TO_TICKS(Rtos_delay));
            
            if (len > 0) { // Proceed only if data was read
                mavlink_message_t msg;
                mavlink_status_t status;
                                 
                for (int i = 0; i < len; ++i) {
                    if (mavlink_parse_char(MAVLINK_COMM_0, tx_buf[i], &msg, &status)) {
                        uint8_t send_buf[MAVLINK_MAX_PACKET_LEN];
                        int send_len = mavlink_msg_to_send_buffer(send_buf, &msg);
                        mysysid=msg.sysid;
                        /* Batch Send */
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
                            
                            // Broadcast to all nodes each 10 seconds
                            if ((msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT)&&(current_time - last_position_broadcast >= POSITION_BROADCAST_INTERVAL)){
                                
                                mavlink_msg_global_position_int_decode(&msg, &myPos);
                                MESH_LOGE("MESH"," GPS Position lat : %d , lon : %d",myPos.lat,myPos.lon);                               
                                auto nodes = mesh_addresses.getlistNodes();
 
                                for (const auto& node : nodes) {
                                   
                                    if  ((!node.is_root)&&(!mesh_addresses.isSelfAddress(node.addr))){
                                        MESH_LOGE("MESH"," Data send to node " MACSTR "  successfully %d ",MAC2STR(node.addr.addr),msg.msgid);
                                        esp_err_t err = esp_mesh_send(&node.addr, &data, MESH_DATA_P2P, NULL, 0);
                                        if (err != ESP_OK) {
                                            MESH_LOGE("MESH", " Error broadcasting position: %d", err);
                                        }
                                    } 
                                }
                                  
                                last_position_broadcast = current_time;
                            }
                            
                            esp_err_t err = esp_mesh_send(&root_address, &data, MESH_DATA_P2P, NULL, 0);
                            if (err != ESP_OK) {
                               MESH_LOGE("MESH"," Error sending data: %d", err);
                            } 
                            offset += packetSize;

                            // Break if error threshold is reached
                            if (error_count >= ERROR_THRESHOLD) {
                                MESH_LOGE("MESH", "Too many errors, stopping task");
                                is_running = false;
                                break;
                            }
                        }
                    }
                }
            } else if (len < 0) {
                MESH_LOGE("UART", "Error reading from UART: %d", len);
            }
        }   
        vTaskDelay(1); // Yield to other tasks
    }
    // Clean up task when done
    vTaskDelete(NULL);
}

void esp_mesh_p2p_rx_main(void *arg)
{
    is_running = true;
    mesh_addr_t from;
    mesh_data_t data;
    int flag = 0;
    wifi_ap_record_t ap_info;
    // Buffer for receiving data from mesh
    static uint8_t rx_buf[MESH_PACKET_SIZE];

    // Reassembly buffer for reassembling the full message
    static uint8_t reassemblyBuffer[BUFFER_SIZE];
    static int receivedLength = 0;

    while (is_running) {
        data.data = rx_buf;
        data.size = sizeof(rx_buf);
        
        esp_err_t err = esp_mesh_recv(&from, &data, portMAX_DELAY, &flag, NULL, 0);
        mesh_addresses.NodeSeen(from);
        if (err != ESP_OK) {
            MESH_LOGE("MESH", "Error receiving data: %d", err);
            error_count++;
            if (error_count >= ERROR_THRESHOLD) {
                MESH_LOGE("MESH", "Too many errors, stopping task");
                is_running = false;
            }
            continue;
        } 

        if (data.size >= sizeof(uint8_t) && data.data[0] == 0x01) {
            esp_wifi_sta_get_ap_info(&ap_info);
            childManager.processReceivedChildInfo(data.data, data.size,ap_info.rssi);
            mesh_addresses.printNodes();
        } else {
            // Check if the reassembly buffer has enough space
            if (receivedLength + data.size <= BUFFER_SIZE) {
                // Copy received data into the reassembly buffer
                memcpy(reassemblyBuffer + receivedLength, data.data, data.size);
                receivedLength += data.size;

                mavlink_message_t message;
                mavlink_status_t status;

                // Try to parse a full Mavlink message from the reassembly buffer
                int bytesParsed = 0;
                for (int i = 0; i < receivedLength; i++) {
                    if (mavlink_parse_char(MAVLINK_COMM_0, reassemblyBuffer[i], &message, &status)) {
                        if ((message.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT)) {
                            MESH_LOGE("MESH", "mysysid: %d message.sysid: %d message.msgid: %d",mysysid,message.sysid,message.msgid);
                            mavlink_global_position_int_t dataP;
                            mavlink_msg_global_position_int_decode(&message, &dataP);
                            if ((myPos.lat!=0)&&(myPos.lon!=0)&&(dataP.lat!=0)&&(dataP.lon!=0)&& ((myPos.lat!=dataP.lat)||(myPos.lon!=dataP.lon))){
                                MESH_LOGE("MESH", "mylatitude: %d", myPos.lat);
                                MESH_LOGE("MESH", "mylongitude: %d", myPos.lon);
                                MESH_LOGE("MESH", "latitude: %d", dataP.lat);
                                MESH_LOGE("MESH", "longitude: %d", dataP.lon);
                                MESH_LOGE("MESH", "DistanceBetween: %f meters",calculate_distance_between_positions(&myPos,&dataP) ); 
                            }                          
                        }else{
                            // Successfully parsed a message, send it out via UART
                            uint8_t send_buf[MAVLINK_MAX_PACKET_LEN];
                            int send_len = mavlink_msg_to_send_buffer(send_buf, &message);
                            uart_write_bytes(CONFIG_UART_PORT_NUM, send_buf, send_len);
                        }
                        // Mark the number of bytes successfully parsed
                        bytesParsed = i + 1;
                    }
                }

            // Shift remaining bytes in the buffer
            if (bytesParsed > 0) {
                memmove(reassemblyBuffer, reassemblyBuffer + bytesParsed, receivedLength - bytesParsed);
                receivedLength -= bytesParsed;
            }
            } else {
                // Buffer overflow, reset the reassembly buffer
                MESH_LOGE("MESH", "Buffer overflow detected. Message too large.");
                receivedLength = 0;
            }
        }
        vTaskDelay(1); // Yield to other tasks
    }

    // Task clean-up
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

uint64_t getMeshAddr(mesh_addr_t* addr) {
    uint64_t result = 0;
    for (int i = 0; i < 6; i++) {
        result = (result << 8) | addr->addr[i];
    }
    return result;
}

void mesh_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data)
{
    mesh_addr_t id = {0,};
    static uint16_t last_layer = 0;
    MESH_LOGE("MESH", " EVENT_MESH ID: %d",event_id);
    switch (event_id) {
    case MESH_EVENT_STARTED: {
        mesh_timing.mesh_start_time = millis();
        esp_mesh_get_id(&id);
        MESH_LOGE("MESH", " <MESH_EVENT_MESH_STARTED>ID: " MACSTR " ", MAC2STR(id.addr));
        is_mesh_connected = false;           
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
        handleConnection(mesh_timing.child);
        mesh_addr_t child_addr;
        memcpy(child_addr.addr, child_connected->mac, 6);
        
        // Obtenir la couche de l'enfant
        int child_layer = esp_mesh_get_layer() + 1;
          // Obtenir le RSSI de l'enfant
        wifi_ap_record_t ap_info;
        esp_wifi_sta_get_ap_info(&ap_info);
        int rssi = ap_info.rssi;
        // Mettre à jour la liste des adresses
        mesh_addresses.updateNode(child_addr, child_layer,false,false,true,rssi);
        
        MESH_LOGE("MESH", "<MESH_EVENT_CHILD_CONNECTED>aid: %d , " MACSTR "",
                 child_connected->aid,
                 MAC2STR(child_connected->mac));
    }
    break;
    case MESH_EVENT_CHILD_DISCONNECTED: {
        mesh_event_child_disconnected_t *child_disconnected = (mesh_event_child_disconnected_t *)event_data;
        handleDisconnection(mesh_timing.child);
        MESH_LOGE("MESH", " <MESH_EVENT_CHILD_DISCONNECTED>aid: %d , " MACSTR "",
                 child_disconnected->aid,
                MAC2STR(child_disconnected->mac));
                      
        mesh_addr_t child_addr;
        memcpy(child_addr.addr, child_disconnected->mac, 6);
        
        // Supprimer le nœud de la liste
        mesh_addresses.removeNode(child_addr);
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
    break;
    case MESH_EVENT_PARENT_CONNECTED: {
        mesh_event_connected_t *connected = (mesh_event_connected_t *)event_data;
        handleConnection(mesh_timing.parent);
        mesh_layer = connected->self_layer;
        
        memcpy(&mesh_parent_addr.addr, connected->connected.bssid, 6);
            // Obtenir le RSSI du parent
        wifi_ap_record_t ap_info;
        esp_wifi_sta_get_ap_info(&ap_info);
        int rssi = ap_info.rssi;
        // Mettre à jour l'adresse du parent
        mesh_addresses.updateParentAddress(mesh_parent_addr);
        
        // Mettre à jour le nœud parent dans la liste
        mesh_addresses.updateNode(mesh_parent_addr, connected->self_layer - 1, esp_mesh_is_root(),false,false,rssi);
        
        esp_mesh_get_id(&id);
        MESH_LOGE("MESH",
                 "<MESH_EVENT_PARENT_CONNECTED>layer: %d --> %d , parent: " MACSTR " %s , ID: " MACSTR " , duty: %d ",
                 last_layer, mesh_layer, MAC2STR(mesh_parent_addr.addr),
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "", MAC2STR(id.addr), connected->duty);
        last_layer = mesh_layer;
        is_mesh_connected = true;
        if (esp_mesh_is_root()) {
            esp_netif_dhcpc_stop(netif_sta);
            esp_netif_dhcpc_start(netif_sta);
        }
        esp_mesh_comm_p2p_start();
        mesh_addresses.updateSelfNode();
        mesh_addresses.updateNode(mesh_addresses.getSelfAddress(), esp_mesh_get_layer(),false,true,true);   
    }
    break;
    case MESH_EVENT_PARENT_DISCONNECTED: {
        mesh_event_disconnected_t *disconnected = (mesh_event_disconnected_t *)event_data;
        handleDisconnection(mesh_timing.parent);
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
           // Supprimer le parent de la liste
        mesh_addresses.removeNode(mesh_parent_addr);

        // Nettoyer la liste des nœuds car la topologie peut avoir changé
        //mesh_addresses.cleanupOnParentDisconnect(); a voir....

        MESH_LOGE("MESH", " <MESH_EVENT_PARENT_DISCONNECTED>reason: %d , layer: %d ",
             disconnected->reason, mesh_layer);
    }
    break;
    case MESH_EVENT_LAYER_CHANGE: {
        mesh_event_layer_change_t *layer_change = (mesh_event_layer_change_t *)event_data;
        mesh_layer = layer_change->new_layer;
    
        MESH_LOGE("MESH", " <MESH_EVENT_LAYER_CHANGE>layer: %d --> %d%s ",
                 last_layer, mesh_layer,
                 esp_mesh_is_root() ? "<ROOT>" :
                (mesh_layer == 2) ? "<layer2>" : "");
        last_layer = mesh_layer;
        mesh_addresses.updateSelfNode();
        mesh_addresses.updateNode(mesh_addresses.getSelfAddress(), esp_mesh_get_layer()+1,false,true,true);
    }
    break;
    case MESH_EVENT_ROOT_ADDRESS: {
        mesh_event_root_address_t *root_addr = (mesh_event_root_address_t *)event_data;
        handleConnection(mesh_timing.root);
        MESH_LOGE("MESH", " <MESH_EVENT_ROOT_ADDRESS>root address: " MACSTR " ",
                 MAC2STR(root_addr->addr));
        memcpy(root_address.addr, root_addr->addr, 6);
               // Mettre à jour la racine dans la liste
        
        mesh_addresses.updateNode(root_address, 1, true);
        MESH_LOGE("MESH", " Root address assigned: " MACSTR " , layer %d ", MAC2STR(root_address.addr),esp_mesh_get_layer()+1);
 
    }
    break;
    case MESH_EVENT_VOTE_STARTED: {
        mesh_event_vote_started_t *vote_started = (mesh_event_vote_started_t *)event_data;
        MESH_LOGE("MESH",
                 "<MESH_EVENT_VOTE_STARTED>attempts: %d , reason: %d , rc_addr: " MACSTR "",
                 vote_started->attempts,
                 vote_started->reason,
                 MAC2STR(vote_started->rc_addr.addr));
    }
    break;
    case MESH_EVENT_VOTE_STOPPED: {
        MESH_LOGE("MESH", " <MESH_EVENT_VOTE_STOPPED> ");
        break;
    }
    case MESH_EVENT_ROOT_SWITCH_REQ: {
        mesh_event_root_switch_req_t *switch_req = (mesh_event_root_switch_req_t *)event_data;
        MESH_LOGE("MESH",
                 " <MESH_EVENT_ROOT_SWITCH_REQ>reason: %d , rc_addr: " MACSTR " ",
                 switch_req->reason,
                 MAC2STR( switch_req->rc_addr.addr));
    }
    break;
    case MESH_EVENT_ROOT_SWITCH_ACK: {
        /* new root */
        mesh_layer = esp_mesh_get_layer();
        esp_mesh_get_parent_bssid(&mesh_parent_addr);
        MESH_LOGE("MESH", " <MESH_EVENT_ROOT_SWITCH_ACK>layer: %d , parent: " MACSTR " " , mesh_layer, MAC2STR(mesh_parent_addr.addr));
    }
    break;
    case MESH_EVENT_TODS_STATE: {
        mesh_event_toDS_state_t *toDs_state = (mesh_event_toDS_state_t *)event_data;
        MESH_LOGE("MESH", " <MESH_EVENT_TODS_REACHABLE>state: %d ", *toDs_state);
    }
    break;
    case MESH_EVENT_ROOT_FIXED: {
        mesh_event_root_fixed_t *root_fixed = (mesh_event_root_fixed_t *)event_data;
        MESH_LOGE("MESH", " <MESH_EVENT_ROOT_FIXED> %s" ,
                 root_fixed->is_fixed ? "fixed" : "not fixed");
    }
    break;
    case MESH_EVENT_ROOT_ASKED_YIELD: {
        mesh_event_root_conflict_t *root_conflict = (mesh_event_root_conflict_t *)event_data;
        MESH_LOGE("MESH",
                 " <MESH_EVENT_ROOT_ASKED_YIELD> " MACSTR " , rssi: %d , capacity: %d" ,
                 MAC2STR(root_conflict->addr),
                 root_conflict->rssi,
                 root_conflict->capacity);
    }
    break;
    case MESH_EVENT_CHANNEL_SWITCH: {
        mesh_event_channel_switch_t *channel_switch = (mesh_event_channel_switch_t *)event_data;
        MESH_LOGE("MESH", "<MESH_EVENT_CHANNEL_SWITCH>new channel: %d ", channel_switch->channel);
    }
    break;
    case MESH_EVENT_SCAN_DONE: {
        mesh_event_scan_done_t *scan_done = (mesh_event_scan_done_t *)event_data;
        MESH_LOGE("MESH", "<MESH_EVENT_SCAN_DONE>number: %d ",
                 scan_done->number);
    }
    break;
    case MESH_EVENT_NETWORK_STATE: {
        mesh_event_network_state_t *network_state = (mesh_event_network_state_t *)event_data;
        MESH_LOGE("MESH", "<MESH_EVENT_NETWORK_STATE>is_rootless: %d ",
                 network_state->is_rootless);   
    }
    break;
    case MESH_EVENT_STOP_RECONNECTION: {
        MESH_LOGE("MESH", "<MESH_EVENT_STOP_RECONNECTION>");
    }
    break;
    case MESH_EVENT_FIND_NETWORK: {
        mesh_event_find_network_t *find_network = (mesh_event_find_network_t *)event_data;
        MESH_LOGE("MESH", "<MESH_EVENT_FIND_NETWORK>new channel: %d , router BSSID:" MACSTR "",
                 find_network->channel, MAC2STR(find_network->router_bssid));
    }
    break;
    case MESH_EVENT_ROUTER_SWITCH: {
        mesh_event_router_switch_t *router_switch = (mesh_event_router_switch_t *)event_data;
        MESH_LOGE("MESH", "<MESH_EVENT_ROUTER_SWITCH>new router:%s, channel: %d , " MACSTR " ",
                 router_switch->ssid, router_switch->channel, MAC2STR(router_switch->bssid));
    }
    break;
    case MESH_EVENT_PS_PARENT_DUTY: {
        mesh_event_ps_duty_t *ps_duty = (mesh_event_ps_duty_t *)event_data;
        MESH_LOGE("MESH", "<MESH_EVENT_PS_PARENT_DUTY>duty: %d ", ps_duty->duty);
    }
    break;
    case MESH_EVENT_PS_CHILD_DUTY: {
        mesh_event_ps_duty_t *ps_duty = (mesh_event_ps_duty_t *)event_data;
        MESH_LOGE("MESH", "<MESH_EVENT_PS_CHILD_DUTY>cidx: %d , " MACSTR ", duty: %d ", ps_duty->child_connected.aid-1,
                MAC2STR(ps_duty->child_connected.mac), ps_duty->duty);
    }
    break;
    default:
        MESH_LOGE("MESH", "unknown id:%" PRId32 "", event_id);
        break;
    }
}

void timing_summary_task(void* pvParameters) {
    while (1) {
        printTimingSummary();
        mesh_addresses.printNodes();
        vTaskDelay(pdMS_TO_TICKS(30000)); // Par exemple toutes les 30 secondes
    }
}
static const int CONNECTED_BIT = BIT0;

/*void init_wifi(wifi_mode_t mode)
{
    const uint8_t protocol = WIFI_PROTOCOL_LR;
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(mesh_event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(mode) );
    wifi_event_group = xEventGroupCreate();

    if (mode == WIFI_MODE_STA) {
        ESP_ERROR_CHECK( esp_wifi_set_protocol(WIFI_IF_STA, protocol) );
        wifi_config_t config = {
            .sta = {
                .ssid = ap_name,
                .password = pass,
                .bssid_set = false
            }
        };
        ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &config) );
        ESP_ERROR_CHECK( esp_wifi_start() );
        ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(80));  // Set TX power to 20.5 dBm (maximum)
        ESP_ERROR_CHECK( esp_wifi_connect() );

        xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                            false, true, portMAX_DELAY);
        ESP_LOGI(TAG, "Connected to AP");
    } else {
        ESP_ERROR_CHECK( esp_wifi_set_protocol(WIFI_IF_AP, protocol) );
        wifi_config_t config = {
            .ap = {
                .ssid = ap_name,
                .password = pass,
                .ssid_len = 0,
                .authmode = WIFI_AUTH_WPA_WPA2_PSK,
                .ssid_hidden = false,
                .max_connection = 3,
                .beacon_interval = 100,
            }
        };
        ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_AP, &config) );
        ESP_ERROR_CHECK( esp_wifi_start() );
    }
}*/


void setup() {
    
    uart_set_baudrate(UART_NUM_0, 921600);// communication sortie USB pour les logs

    uart_config_t uart_config = {//communication avec l'autopilote
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(CONFIG_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(CONFIG_UART_PORT_NUM, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));//pin 17 TX2 pin 16 RX2
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

    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(80));  // Set TX power to 20.5 dBm (maximum)
    
    /*  mesh initialization */
    ESP_ERROR_CHECK(esp_mesh_init());
    ESP_ERROR_CHECK(esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler, NULL));
    /*  set mesh topology */
    ESP_ERROR_CHECK(esp_mesh_set_topology(MESH_TOPO_TREE));
    //ESP_ERROR_CHECK(esp_mesh_set_topology(MESH_TOPO_CHAIN));
    /*  set mesh max layer according to the topology */
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(6));
    ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1));
    ESP_ERROR_CHECK(esp_mesh_set_xon_qsize(128));

    /* Disable mesh PS function */
    ESP_ERROR_CHECK(esp_mesh_disable_ps());
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(120));

    /* set this esp as node*/
    ESP_ERROR_CHECK(esp_mesh_set_type(MESH_IDLE));
    /* informs this esp that root is already set */
    ESP_ERROR_CHECK(esp_mesh_fix_root(true));

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
    
    delay(2000);
    mesh_addresses.updateSelfNode();
    mesh_addresses.updateNode(mesh_addresses.getSelfAddress(), esp_mesh_get_layer()+1,false,true,true);
    // Créer la tâche d'affichage des statistiques
    xTaskCreate(timing_summary_task, 
                "timing_summary", 
                4096,           // Stack size 
                NULL,           // Task parameters
                1,             // Priority
                NULL);         // Task handle
}

void loop()
{
    
}