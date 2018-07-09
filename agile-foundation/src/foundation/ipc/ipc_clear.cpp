#include "foundation/utf.h"
#include "foundation/internal/sync.h"

int main1() {
  google::InitGoogleLogging("ipc_clear");
  google::FlushLogFiles(google::GLOG_INFO);
  FLAGS_colorlogtostderr = true;
  google::SetStderrLogging(google::GLOG_INFO);

  LOG_INFO << "Starting to clear shm and msgq... ...";
  internal::__clear(internal::IpcType::N_IPC);

  google::ShutdownGoogleLogging();
  return 0;
}
