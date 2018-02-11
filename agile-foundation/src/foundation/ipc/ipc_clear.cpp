#include "foundation/utf.h"
#include "foundation/internal/sync.h"

int main() {
  google::InitGoogleLogging("ipc_clear");
  google::FlushLogFiles(google::GLOG_INFO);
  FLAGS_colorlogtostderr = true;
  google::SetStderrLogging(google::GLOG_INFO);

  LOG_INFO << "Starting to clear shm and msgq... ...";
  internal::__clear(internal::IPC_TYPE::N_IPC);

  google::ShutdownGoogleLogging();
  return 0;
}
