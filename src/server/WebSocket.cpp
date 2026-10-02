#include "server/WebSocket.h"

#include <algorithm>
#include <cctype>
#include <cstring>

namespace tactics::server::ws {

namespace {
uint32_t Rol(uint32_t v, int n) { return (v << n) | (v >> (32 - n)); }
}  // namespace

std::string Sha1(const std::string& data) {
  uint32_t h[5] = {0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0};
  std::string msg = data;
  msg.push_back(static_cast<char>(0x80));
  while (msg.size() % 64 != 56) msg.push_back('\0');
  const uint64_t bits = static_cast<uint64_t>(data.size()) * 8;
  for (int i = 7; i >= 0; --i) msg.push_back(static_cast<char>((bits >> (i * 8)) & 0xFF));

  for (size_t off = 0; off < msg.size(); off += 64) {
    uint32_t w[80];
    for (int i = 0; i < 16; ++i) {
      w[i] = (static_cast<uint32_t>(static_cast<uint8_t>(msg[off + i * 4])) << 24) |
             (static_cast<uint32_t>(static_cast<uint8_t>(msg[off + i * 4 + 1])) << 16) |
             (static_cast<uint32_t>(static_cast<uint8_t>(msg[off + i * 4 + 2])) << 8) |
             static_cast<uint32_t>(static_cast<uint8_t>(msg[off + i * 4 + 3]));
    }
    for (int i = 16; i < 80; ++i) w[i] = Rol(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
    for (int i = 0; i < 80; ++i) {
      uint32_t f, k;
      if (i < 20) { f = (b & c) | (~b & d); k = 0x5A827999; }
      else if (i < 40) { f = b ^ c ^ d; k = 0x6ED9EBA1; }
      else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDC; }
      else { f = b ^ c ^ d; k = 0xCA62C1D6; }
      const uint32_t tmp = Rol(a, 5) + f + e + k + w[i];
      e = d; d = c; c = Rol(b, 30); b = a; a = tmp;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e;
  }
  std::string out;
  for (int i = 0; i < 5; ++i) {
    for (int s = 24; s >= 0; s -= 8) out.push_back(static_cast<char>((h[i] >> s) & 0xFF));
  }
  return out;
}

std::string Base64Encode(const std::string& in) {
  static const char kAlphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  size_t i = 0;
  for (; i + 2 < in.size(); i += 3) {
    const uint32_t v = (static_cast<uint8_t>(in[i]) << 16) | (static_cast<uint8_t>(in[i + 1]) << 8) |
                       static_cast<uint8_t>(in[i + 2]);
    out.push_back(kAlphabet[(v >> 18) & 63]);
    out.push_back(kAlphabet[(v >> 12) & 63]);
    out.push_back(kAlphabet[(v >> 6) & 63]);
    out.push_back(kAlphabet[v & 63]);
  }
  if (i + 1 == in.size()) {
    const uint32_t v = static_cast<uint8_t>(in[i]) << 16;
    out.push_back(kAlphabet[(v >> 18) & 63]);
    out.push_back(kAlphabet[(v >> 12) & 63]);
    out += "==";
  } else if (i + 2 == in.size()) {
    const uint32_t v = (static_cast<uint8_t>(in[i]) << 16) | (static_cast<uint8_t>(in[i + 1]) << 8);
    out.push_back(kAlphabet[(v >> 18) & 63]);
    out.push_back(kAlphabet[(v >> 12) & 63]);
    out.push_back(kAlphabet[(v >> 6) & 63]);
    out.push_back('=');
  }
  return out;
}

std::string AcceptKey(const std::string& clientKey) {
  return Base64Encode(Sha1(clientKey + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"));
}

namespace {
std::string Lower(std::string s) {
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}
std::string Trim(const std::string& s) {
  size_t a = 0, b = s.size();
  while (a < b && (s[a] == ' ' || s[a] == '\t')) ++a;
  while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\t')) --b;
  return s.substr(a, b - a);
}
}  // namespace

ParseStatus ParseHttpRequest(const std::string& buf, size_t maxBytes, HttpRequest* out,
                             size_t* consumed) {
  const size_t end = buf.find("\r\n\r\n");
  if (end == std::string::npos) return buf.size() > maxBytes ? ParseStatus::Bad : ParseStatus::NeedMore;
  if (end + 4 > maxBytes) return ParseStatus::Bad;
  HttpRequest req;
  size_t pos = 0;
  bool first = true;
  while (pos < end + 2) {
    const size_t eol = buf.find("\r\n", pos);
    const std::string line = buf.substr(pos, eol - pos);
    pos = eol + 2;
    if (first) {
      first = false;
      const size_t s1 = line.find(' ');
      const size_t s2 = s1 == std::string::npos ? s1 : line.find(' ', s1 + 1);
      if (s2 == std::string::npos) return ParseStatus::Bad;
      req.method = line.substr(0, s1);
      req.path = line.substr(s1 + 1, s2 - s1 - 1);
      continue;
    }
    const size_t colon = line.find(':');
    if (colon == std::string::npos) return ParseStatus::Bad;
    const std::string name = Lower(Trim(line.substr(0, colon)));
    const std::string value = Trim(line.substr(colon + 1));
    if (name == "upgrade") req.upgrade = Lower(value);
    else if (name == "sec-websocket-key") req.key = value;
    else if (name == "sec-websocket-version") req.version = value;
  }
  *out = std::move(req);
  *consumed = end + 4;
  return ParseStatus::Ok;
}

std::string HandshakeResponse(const std::string& clientKey) {
  return "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
         "Sec-WebSocket-Accept: " + AcceptKey(clientKey) + "\r\n\r\n";
}

std::string HttpTextResponse(int status, const std::string& reason, const std::string& body) {
  return "HTTP/1.1 " + std::to_string(status) + " " + reason +
         "\r\nContent-Type: text/plain\r\nConnection: close\r\nContent-Length: " +
         std::to_string(body.size()) + "\r\n\r\n" + body;
}

std::string EncodeFrame(Opcode op, const std::string& payload) {
  std::string out;
  out.push_back(static_cast<char>(0x80 | static_cast<uint8_t>(op)));
  const size_t n = payload.size();
  if (n < 126) {
    out.push_back(static_cast<char>(n));
  } else if (n <= 0xFFFF) {
    out.push_back(126);
    out.push_back(static_cast<char>((n >> 8) & 0xFF));
    out.push_back(static_cast<char>(n & 0xFF));
  } else {
    out.push_back(127);
    for (int i = 7; i >= 0; --i) out.push_back(static_cast<char>((static_cast<uint64_t>(n) >> (i * 8)) & 0xFF));
  }
  out += payload;
  return out;
}

std::string EncodeClose(uint16_t code) {
  std::string payload;
  payload.push_back(static_cast<char>(code >> 8));
  payload.push_back(static_cast<char>(code & 0xFF));
  return EncodeFrame(Opcode::Close, payload);
}

std::optional<Message> FrameParser::Next() {
  while (!failed_) {
    if (buf_.size() < 2) return std::nullopt;
    const uint8_t b0 = static_cast<uint8_t>(buf_[0]);
    const uint8_t b1 = static_cast<uint8_t>(buf_[1]);
    const bool fin = b0 & 0x80;
    const uint8_t op = b0 & 0x0F;
    if (b0 & 0x70) { Fail(1002); break; }       // RSV bits: no extensions negotiated.
    if (!(b1 & 0x80)) { Fail(1002); break; }    // Clients must mask.
    uint64_t len = b1 & 0x7F;
    size_t header = 2;
    if (len == 126) {
      if (buf_.size() < 4) return std::nullopt;
      len = (static_cast<uint8_t>(buf_[2]) << 8) | static_cast<uint8_t>(buf_[3]);
      header = 4;
    } else if (len == 127) {
      if (buf_.size() < 10) return std::nullopt;
      len = 0;
      for (int i = 0; i < 8; ++i) len = (len << 8) | static_cast<uint8_t>(buf_[2 + i]);
      header = 10;
    }
    const bool control = op >= 8;
    if (len > maxMessageBytes_ || (control && (len > 125 || !fin))) {
      Fail(control ? 1002 : 1009);
      break;
    }
    if (buf_.size() < header + 4 + len) return std::nullopt;
    const char* mask = buf_.data() + header;
    std::string payload(buf_.data() + header + 4, static_cast<size_t>(len));
    for (size_t i = 0; i < payload.size(); ++i) payload[i] ^= mask[i % 4];
    buf_.erase(0, header + 4 + static_cast<size_t>(len));

    if (control) {
      if (op != 8 && op != 9 && op != 10) { Fail(1002); break; }
      return Message{static_cast<Opcode>(op), std::move(payload)};
    }
    if (op == 0) {
      if (!inFragment_) { Fail(1002); break; }
    } else if (op == 1 || op == 2) {
      if (inFragment_) { Fail(1002); break; }
      fragmentOp_ = static_cast<Opcode>(op);
      fragments_.clear();
      inFragment_ = true;
    } else {
      Fail(1002);
      break;
    }
    fragments_ += payload;
    if (fragments_.size() > maxMessageBytes_) { Fail(1009); break; }
    if (fin) {
      inFragment_ = false;
      return Message{fragmentOp_, std::move(fragments_)};
    }
  }
  return std::nullopt;
}

}  // namespace tactics::server::ws
