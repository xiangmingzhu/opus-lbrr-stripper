# opus-lbrr-stripper
Remove LBRR sub-frame in a Opus 20ms + silk-only + mono frame.
The approach is straightforward: range decode -> skip LBRR symbols -> range encode
