#include "server/JevBot.h"

#include <algorithm>
#include <utility>

#include "net/Json.h"
#include "server/HttpClient.h"

namespace tactics::server {

using net::Json;

JevBotRunner::JevBotRunner(JevBotConfig config, Poster poster)
    : config_(std::move(config)), poster_(std::move(poster)) {
  if (!poster_) {
    HttpUrl url;
    if (config_.adapterUrl.empty() || !ParseHttpUrl(config_.adapterUrl, &url)) return;
    poster_ = [this, url](const std::string& body, int* status, std::string* reply,
                          std::string* error) {
      HttpResponse response;
      if (!HttpPostJson(url, "/v1/choice", body, config_.timeoutMs, &response, error,
                        [this] { return stopping_.load(); })) {
        return false;
      }
      *status = response.status;
      *reply = std::move(response.body);
      return true;
    };
  }
  enabled_ = true;
  for (int i = 0; i < std::max(1, config_.workers); ++i) threads_.emplace_back([this] { Work(); });
}

JevBotRunner::~JevBotRunner() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  wake_.notify_all();
  for (std::thread& t : threads_) t.join();
}

void JevBotRunner::Submit(JevJob job) {
  if (!enabled_) return;
  ++pending_;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    jobs_.push_back(std::move(job));
  }
  wake_.notify_one();
}

std::vector<JevResult> JevBotRunner::TakeResults() {
  std::lock_guard<std::mutex> lock(mutex_);
  return std::exchange(results_, {});
}

void JevBotRunner::Work() {
  for (;;) {
    JevJob job;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait(lock, [this] { return stopping_ || !jobs_.empty(); });
      if (stopping_) return;
      job = std::move(jobs_.front());
      jobs_.pop_front();
    }
    JevResult result = Run(job);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      results_.push_back(std::move(result));
    }
    --pending_;
  }
}

JevResult JevBotRunner::Run(JevJob& job) {
  JevResult result;
  result.roomId = job.roomId;
  result.roomName = job.roomName;
  result.team = job.team;
  result.fallback = job.fallback;

  ai::JevRequestBuilder builder(job.game, job.team, job.nonce);
  while (!builder.Step(25.0)) {
    if (stopping_) {
      result.error = "server is shutting down";
      return result;
    }
  }
  std::optional<ai::JevRequest> request = builder.Finish();
  if (!request) {
    result.nothingToAsk = true;
    return result;
  }
  result.request = *request;
  if (job.fallback) {
    result.choices = ai::DeterministicFallbackChoice(*request);
    return result;
  }
  int status = 0;
  std::string body, error;
  if (!poster_(request->json, &status, &body, &error)) {
    result.error = error.empty() ? "Jev request failed" : error;
    return result;
  }
  result.choices = ParseReply(*request, status, body, &result.error);
  return result;
}

std::vector<std::string> JevBotRunner::ParseReply(const ai::JevRequest& request, int status,
                                                  const std::string& body, std::string* error) {
  Json reply;
  const bool parsed = Json::Parse(body, &reply) && reply.IsObject();
  if (status != 200) {
    const bool hasMessage = parsed && reply["error"].IsString() && !reply["error"].AsString().empty();
    *error = hasMessage ? "Jev: " + reply["error"].AsString()
                        : "Jev adapter returned HTTP " + std::to_string(status);
    return {};
  }
  if (!parsed || !reply["choices"].IsObject()) {
    *error = "Jev adapter returned an invalid reply";
    return {};
  }
  const Json& choices = reply["choices"];
  std::vector<std::string> out;
  for (int actor : request.actorIds) {
    const Json& pick = choices[std::to_string(actor)];
    if (!pick.IsString()) {
      *error = "Jev adapter reply is missing a figure's choice";
      return {};
    }
    out.push_back(pick.AsString());
  }
  return out;
}

}  // namespace tactics::server
