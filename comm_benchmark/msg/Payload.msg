# 1 KiB benchmark payload. Field layout matches the C struct used on the
# raw-UDP transport so the wire size is identical (1024 B).
#
# Header (24 B) — measurement metadata.
uint32 own_seq                # this side's monotonic sequence
uint32 peer_last_seq          # peer's seq the sender most recently received
int64  own_send_ts_ns         # sender's CLOCK_MONOTONIC at send
int64  peer_recv_ts_ns        # sender's CLOCK_MONOTONIC at recv of peer_last_seq

# Data (144 B) — bilateral-equivalent state.
float64[6] q
float64[6] qd
float64[6] tau_ext

# Padding (856 B) — pads payload to exactly 1 KiB so that bandwidth and
# fragmentation behaviour matches the worst-case bilateral message.
uint8[856] padding
