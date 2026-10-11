# Proposed GitHub metadata

Apply these manually after review. Remote settings have not been changed.

## Description

Dual-clock Vector MAC with multi-outstanding AXI read DMA, RID-indexed ROB, CDC/formal verification, and a Nangate45 OpenROAD RTL-to-GDS research implementation.

## Topics

`asic`, `rtl`, `verilog`, `systemverilog`, `axi`, `dma`, `reorder-buffer`,
`clock-domain-crossing`, `formal-verification`, `symbiyosys`, `openroad`,
`rtl-to-gds`, `physical-design`, `static-timing-analysis`, `hardware-accelerator`,
`nangate45`

<a id="p10-review"></a>

## Description and navigation rationale

The [published repository metadata](https://api.github.com/repos/szuwei-yeh/mac-accel-cdc)
reviewed for commit `5a45665` still describes the earlier Nexys A7 implementation
and has no topics. Updating the description to identify the V3 DMA/ROB and
Nangate45 research implementation is a material improvement. The topic set above
already covers the implemented design; apply that set without adding more topics.

The main README already presents four scoped accomplishments before architecture,
optimization, physical evidence and verification. Retain that hierarchy and use
standard implementation-stage terminology in reader-facing text and figures.
Detailed physical decisions, measurement conditions and verification limits
remain linked from the main README. No LICENSE is added by this polish pass.
