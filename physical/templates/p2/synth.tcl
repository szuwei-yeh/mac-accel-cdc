yosys scratchpad -set abc.exe @TOOLS@/bin/abc-batch
source @FLOW@/scripts/synth.tcl
yosys write_json $::env(P2_RUN)/audit/mapped.json
exec /usr/bin/python3 $::env(P2_PHASE)/audit_mapping.py
