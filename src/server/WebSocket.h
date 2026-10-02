#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace tactics::server::ws {

// Minimal RFC 6455 server-side pieces (handshake + framing); the server
// carries no third-party networking dependency.

std::string Sha1(const std::string& data);  // 20 raw bytes.
std::string Base64Encode(const std::string& data);
std::string AcceptKey(const std::string& clientKey);

struct HttpRequest {
  std::string method;
  std::string path;
  std::string upgrade;  // Lowercased Upgrade header.
  std::string key;      // Sec-WebSocket-Key.
  std::string version;  // Sec-WebSocket-Version.
};
enum class ParseStatus { NeedMore, Ok, Bad };
// Parses a request head out of `buf` (terminated by CRLFCRLF). On Ok,
// *consumed is the head's byte length. Heads over `maxBytes` are Bad.
ParseStatus ParseHttpRequest(const std::string& buf, size_t maxBytes, HttpRequest* out,
                             size_t* consumed);
std::string HandshakeResponse(const std::string& clientKey);
std::string HttpTextResponse(int status, const std::string& reason, const std::string& body);

enum class Opcode : uint8_t { Continuation = 0, Text = 1, Binary = 2, Close = 8, Ping = 9, Pong = 10 };

std::string EncodeFrame(Opcode op, const std::string& payload);
std::string EncodeClose(uint16_t code);

struct Message {
  Opcode op = Opcode::Text;
  std::string payload;  // For Close: the raw payload (status code + reason).
};

// Incremental client-frame parser: feed received bytes, then pull complete
// messages (fragments are reassembled). Enforces masking and a size cap.
class FrameParser {
 public:
  explicit FrameParser(size_t maxMessageBytes) : maxMessageBytes_(maxMessageBytes) {}
  void Feed(const char* data, size_t n) { buf_.append(data, n); }
  // Returns the next complete message, or nullopt if more bytes are needed
  // or the stream is invalid (check failed()).
  std::optional<Message> Next();
  bool failed() const { return failed_; }
  // WebSocket close code to send for a failure (1002 protocol, 1009 too big).
  uint16_t failure_code() const { return failureCode_; }

 private:
  bool Fail(uint16_t code) {
    failed_ = true;
    failureCode_ = code;
    return false;
  }
  std::string buf_;
  std::string fragments_;
  Opcode fragmentOp_ = Opcode::Text;
  bool inFragment_ = false;
  size_t maxMessageBytes_;
  bool failed_ = false;
  uint16_t failureCode_ = 1002;
};

}  // namespace tactics::server::ws
