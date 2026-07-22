#ifndef _RASPIKE_PROTOCOL_API_H_
#define _RASPIKE_PROTOCOL_API_H_

#include "raspike_com.h"
#include "raspike_protocol_com.h"

#ifdef __cplusplus
extern "C" {
#endif  

typedef struct {
  uint32_t session_id;
  uint8_t protocol_version;
  uint8_t capabilities;
  uint16_t reserved;
  uint64_t tx_frames;
  uint64_t rx_frames;
  uint64_t crc_errors;
  uint64_t framing_errors;
  uint64_t session_errors;
  uint64_t request_timeouts;
  uint64_t request_retries;
  uint64_t stale_acks;
} RasPikeLinkStats;

  extern int raspike_prot_init(RPComDescriptor *desc);
  extern int raspike_prot_receive(void);
  extern int raspike_prot_shutdown(void);
  extern int raspike_drive_configure(void);
  extern int raspike_drive_send_command(const RPRealtimeDriveCommand *command);
  extern int raspike_runtime_inject_fault(uint32_t fault_mask,
                                          uint32_t duration_cycles);
  extern int raspike_runtime_get_telemetry(RPRuntimeTelemetry *telemetry);
  extern int raspike_link_get_stats(RasPikeLinkStats *stats);
  
#ifdef __cplusplus
}
#endif

#endif

