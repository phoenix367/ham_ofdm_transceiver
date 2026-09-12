# Board integration benchmark (2026-09-13 01:51)

A = 240041000551333438363436, B = 320047000851333438363436; 887 s total

| scenario | result | seconds | summary |
|---|---|---|---|
| attach | ok | 1.5 | A: rung 10 SNR -- (none in 60 s) peer_state 3 die 46.1 C; B: rung 10 SNR -- (none in 60 s) peer_state 3 die 44.3 C |
| warmup | ok | 10.2 | A->B: 3.2 s, handshake 5.2 s, rung 12, SNR 24.7 dB; B->A: 1.4 s, handshake 3.4 s, rung 12, SNR 22.5 dB |
| message | ok | 23.8 | A->B: [1.0, 1.1, 1.0] s; B->A: [1.1, 1.1, 1.0] s |
| chat | ok | 10.0 | all 5.2 s, arrivals {'A->B': [0.9, 2.7, 4.5], 'B->A': [1.8, 3.6, 5.5]} |
| bulk | ok | 17.4 | A->B: [2.4, 2.5] s; B->A: [2.5, 2.5] s |
| file | ok | 111.5 | A->B: 6000 B in 53.4 s (112.4 B/s), exact True, tx 3 to 0 retx 0; B->A: 6000 B in 52.6 s (114.0 B/s), exact True, tx 3 to 0 retx 0 |
| file_bidir | ok | 91.1 | 3000 B each way, both in 87.5 s (68.6 B/s aggregate) |
| file_chat | ok | 73.1 | file A->B, chat B->A: file 26.4 s (113.5 B/s), chat latency [25.1, 18.7, 12.6]; file B->A, chat A->B: file 28.4 s (105.7 B/s), chat latency [26.9, 20.6, 14.5] |
| bcast | ok | 19.2 | A->B: 1.8 s, 4 ok / 0 lost, exact True; B->A: 11.7 s, 4 ok / 0 lost, exact True |
| bcastfile | ok | 191.7 | A->B: 4096/4096 B in 92.1 s, 175 ok / 0 lost; B->A: 4096/4096 B in 94.1 s, 175 ok / 0 lost |
| bcast_msg | ok | 211.2 | bcast A->B, msg B->A: bcast 92.8 s (0 lost), reply queued at 9.5 s arrived 18.1 s after the end; bcast B->A, msg A->B: bcast 80.5 s (0 lost), reply queued at 1.0 s arrived 18.3 s after the end |
| speech | ok | 129.3 | A->B: warm 0.99 s, first audio out 3.04 s, 41.12 s received, 0.0 % lost, 70 ok / 0 lost; B->A: warm 6.74 s, first audio out 3.02 s, 40.16 s received, 2.3 % lost, 65 ok / 2 lost |
| speech_file | ok | 108.6 | A->B: 105.1 s (57.1 B/s) |
| stale | ok | 126.0 | idle 120.0 s, rung {'A': 12, 'B': 12} -> {'A': 11, 'B': 11}, message 1.0 s, rung after {'A': 12, 'B': 10} |
