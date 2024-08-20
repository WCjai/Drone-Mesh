#ifndef _PAINLESS_MESH_CONNECTION_HPP_
#define _PAINLESS_MESH_CONNECTION_HPP_

#include <list>

#include "Arduino.h"
#include "painlessmesh/configuration.hpp"

#include "painlessmesh/buffer.hpp"
#include "painlessmesh/logger.hpp"

extern painlessmesh::logger::LogClass Log;

namespace painlessmesh {
namespace tcp {

// Shared buffer for reading/writing to the buffer
static painlessmesh::buffer::temp_buffer_t shared_buffer;

/**
 * Class that performs buffered read and write to the tcp connection
 * (asyncclient)
 *
 * Note that the class expects to be wrapped in a shared_ptr to ensure proper
 * lifetime management. Objects generally will only be destroyed after close is
 * called (and the reading and writing tasks are stopped).
 */
class BufferedConnection
    : public std::enable_shared_from_this<BufferedConnection> {
 public:
  /**
   * Create a buffered connection around the client
   *
   *
   * BufferedConnection takes ownership of the client  and will delete the
   * pointer at the end.
   *
   * One should always call initialize after construction
   */
  BufferedConnection(AsyncClient *client) : client(client) {}

  ~BufferedConnection() {
    this->close();
    if (!client->freeable()) {
      client->close(true);
    }
    client->abort();
    delete client;
  }

  void initialize(Scheduler *scheduler) {
    auto self = this->shared_from_this();
    sentBufferTask.set(
        TASK_SECOND, TASK_FOREVER, [self]() {
          if (!self->sentBuffer.empty() && self->client->canSend()) {
            auto ret = self->writeNext();
            if (ret)
              self->sentBufferTask.forceNextIteration();
            else
              self->sentBufferTask.delay(100 * TASK_MILLISECOND);
          }
        });
    scheduler->addTask(sentBufferTask);
    sentBufferTask.enableDelayed();

    readBufferTask.set(TASK_SECOND, TASK_FOREVER,
                       [self]() {
                         if (!self->receiveBuffer.empty()) {
                           TSTRING frnt = self->receiveBuffer.front();
                           self->receiveBuffer.pop_front();
                           if (!self->receiveBuffer.empty())
                             self->readBufferTask.forceNextIteration();
                           if (self->receiveCallback)
                             self->receiveCallback(frnt);
                         }
                       });
    scheduler->addTask(readBufferTask);
    readBufferTask.enableDelayed();

    client->onAck(
        [self](void *arg, AsyncClient *client,
                                          size_t len, uint32_t time) {
          self->sentBufferTask.forceNextIteration();
        },
        NULL);

    client->onData(
        [self](void *arg, AsyncClient *client,
                                          void *data, size_t len) {
          self->receiveBuffer.push(static_cast<const char *>(data), len,
                                   shared_buffer);
          // Signal that we are done
          self->client->ack(len);
          self->readBufferTask.forceNextIteration();
        },
        NULL);

    client->onDisconnect([self](
                             void *arg, AsyncClient *client) { self->close(); },
                         NULL);
  }

  void close() {
    if (!mConnected) return;

    // Disable tasks and callbacks
    this->sentBufferTask.setCallback(NULL);
    this->sentBufferTask.disable();
    this->readBufferTask.setCallback(NULL);
    this->readBufferTask.disable();

    this->client->onData(NULL, NULL);
    this->client->onAck(NULL, NULL);
    this->client->onDisconnect(NULL, NULL);
    this->client->onError(NULL, NULL);

    if (client->connected()) {
      client->close();
    }

    receiveBuffer.clear();
    sentBuffer.clear();

    if (disconnectCallback) disconnectCallback();

    receiveCallback = NULL;
    disconnectCallback = NULL;

    mConnected = false;
  }

  bool write(TSTRING data, bool priority = false) {
    sentBuffer.push(data, priority);
    sentBufferTask.forceNextIteration();
    return true;
  }

  void onDisconnect(std::function<void()> callback) {
    disconnectCallback = callback;
  }

  void onReceive(std::function<void(TSTRING)> callback) {
    receiveCallback = callback;
  }

  bool connected() { return mConnected; }

 protected:
  bool mConnected = true;

  AsyncClient *client;

  std::function<void(TSTRING)> receiveCallback;
  std::function<void()> disconnectCallback;

  painlessmesh::buffer::ReceiveBuffer<TSTRING> receiveBuffer;
  painlessmesh::buffer::SentBuffer<TSTRING> sentBuffer;

  bool writeNext() {
    if (sentBuffer.empty()) {
      return false;
    }
    auto len = sentBuffer.requestLength(shared_buffer.length);
    auto snd_len = client->space();
    if (len > snd_len) len = snd_len;
    if (len > 0) {
      // sentBuffer.read(len, shared_buffer);
      // auto written = client->write(shared_buffer.buffer, len, 1);
      auto data_ptr = sentBuffer.readPtr(len);
      auto written = client->write(data_ptr, len, 1);
      if (written == len) {
        client->send();  // TODO only do this for priority messages
        sentBuffer.freeRead();
        sentBufferTask.forceNextIteration();
        return true;
      } else if (written == 0) {
        return false;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

  Task sentBufferTask;
  Task readBufferTask;

  template <typename T>
  std::shared_ptr<T> shared_from(T *derived) {
    return std::static_pointer_cast<T>(shared_from_this());
  }
};  // namespace tcp
};  // namespace tcp
};  // namespace painlessmesh

#endif
