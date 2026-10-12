# Final RTL correctness closure

**RTL functional closure PASS；P&R qualification BLOCKED。** 三項已確認功能 defects 已修復，支援範圍內無剩餘已確認 RTL defect；此結論不代表 bug-free。V3 synthesis input 已改變，qualified6／舊 mapped GLS 不再綁定最終 RTL，故本輪不回報「RTL READY FOR P&R within the documented supported scope」。

工作區：`/private/tmp/vector-mac-integration-rdj_puzu`；分支 `integrate/rtl-physical-portfolio-20261012`；Git 基準／HEAD `762b8fae899d5b1f64183d91a30b3d58c34f2188`。原審查報告與交接包原樣保留，本報告取代其當前來源／readiness 結論。本次沒有 physical experiment、staging、commit、merge 或 push。

## Findings 與最小修正

| Finding | 同一最終 TB 的 pre-fix 證據 | 修正與 post-fix |
|---|---|---|
| C01：V1 disabled bytes 被清零，zero/upper-only writes 仍改 MASK／operand index | `tb_mac_accel_axi_wstrb`：9 PASS / 9 FAIL | [wrapper](rtl/mac_accel_axi.v#L161) 傳遞 saved WSTRB；[core](rtl/mac_accel.v#L176) 逐 byte 保留 storage 並限定 side effects。18/18 PASS，4 jobs |
| C02：V2 CSR commit 忽略 captured WSTRB | `tb_mac_accel_dma_axil_wstrb`：7 PASS / 19 FAIL | [V2 commit](rtl/mac_accel_dma_top.v#L165) 地址／LENGTH byte enables，CTRL byte0 gating；26/26 PASS |
| C03：V3 DMA idle start 丟棄上一 job stalled final beat | `tb_mac_dma_rob_pending_tail`：先觀察 dma_done=1、OUT=0、pending=1，下一合法 idle start 觸發 `pending final beat discarded by idle start` | [DMA start](rtl/mac_dma_rob.v#L192) 只移除清除 stream_valid 的一行；保留 skid ownership 至接受，RUN_B 等待 room。2 jobs、4 AR／4 R／2 OUT、36 held cycles、error→clean PASS；V2 parity TB 亦 PASS |

Pre/post 使用相同 TB bytes；`exact-pre-fix-results.json` 保存三份 TB SHA 與原 RTL replay 路徑。沒有改既有 expected values。三版本獨立 expected-sum corruption controls 都被最終來源正確拒絕。

## Coverage matrix

逐讀 RTL、實際 stimulus 與 assertions 後分類；未觸發的條件不計 coverage。

| 審查項目 | 最終分類 | 有效證據／界線 |
|---|---|---|
| start/done/busy/error ownership、連續 jobs | 已有有效測試（含本輪補測） | V1 兩方向 toggle；V2/V3 各 seed 12 jobs 無 inter-job reset、start/done/CSR/FIFO counts；V3 busy suite 15 lifecycle cases。V1/V2 是既有軟體單 job 契約，BUSY 低不等於 completion |
| error recovery、final A/B beat error | 已有有效測試 | 新 V2/V3 stress 每 seed 8 個 final-beat faults，error→clean；既有 V3 error suite 12 scenarios／26 jobs 並要求 fault/gap/full counters |
| AW-before-W、W-before-AW、讀寫交錯、B/R stalls | 已有有效測試 | latency/skew、payload poison、response stability、外部 AW/W/AR queues；V2/V3 新 stream TB 各 4 modes、1024 writes＋1024 reads，held VALID 下地址／資料／WSTRB 均變化 |
| 各版本 full／zero／partial WSTRB | 已有有效測試 | V1 新 directed18、V2 directed26、既有 V3 directed26；索引／MASK／disabled bytes／CTRL 各自 assertion |
| credits、ID reuse、非 head completion、ordered retirement | 已有有效測試 | ROB completion TB 真正先完成 ID7、阻止 receive-complete／partial-drain 提早還 credit；20 個 full32-bit ordered beats、8→10 accepted AR；stress 實際 non-head completion counters 非零 |
| FIFO full/empty、last item 與 completion 順序 | 已有有效測試 | payload/last/order、accepted write/read conservation、Gray transitions；慢 MAC 必須觸發 full/stall；C03 補測 final skid 跨 idle start。CTRL DONE 與 block dma_done 的契約分開 |
| CDC toggle 兩翻轉、bundle hold、clock ratios | 已有有效測試 | V3 六 scenario／36 jobs；新三版本 7.5／30／3.5 ns MAC clocks、每 run 12 jobs；source result/latency 保持至下一 start 的 assertion |
| startup reset release order／coordinated fixture reset | 已有有效測試 | V3 CDC 不同 release order，兩 clock domain 正常運行；不等於 RDC signoff |
| signed extrema／overflow、長度與 partial final burst | 已有有效測試 | 獨立 signed64 reference→low32；−32768／32767、modulo overflow；lengths 1,15,16,17,31,32,33,127,129,255,256,1；V1 MASK 0/5/A/F |
| driver／latch／索引與寬度／deadlock | 已有有效測試＋結構審查 | Yosys check -assert／no-dlatch，Verilator、bounded progress；qualified index 範圍 V1 0..3、DMA 0..255；permanent external stalls 不保證 completion |
| MAX_BURST_LEN=1、任意參數 | 明確不支援 | 1 的 zero-width beat slice：Icarus elaboration exit2，非 functional PASS；未宣稱其他任意組合可用 |
| 跨 4KB burst、unaligned DMA | 明確不支援 | 無 split/alignment check；caller 保證每 burst 合法且 source 四 byte 對齊；新 stress 測到 fc0 burst 恰至頁尾與下一頁合法 burst |
| 單邊 reset、active-job abort／stopped-clock recovery | 明確不支援／尚未驗證 | 單邊 reset 不屬支援 recovery；live peer 的 active abort、stopped-clock recovery 需補測，不標 PASS |
| LATENCY 32-bit wrap、full-top equivalence、MTBF/RDC／SDF | 需補測試／未建立 | 不在本輪功能／數位 CDC closure 的 demonstrated scope |

[完整介面契約](docs/RTL_SUPPORTED_CONTRACT.md) 固定 V1 DATA16/VEC4、V2 DATA16/AXI32/ID4/MAX_LEN256/BURST16、V3 outstanding8；包括地址 alias、MASK/index byte semantics、單 job 軟體協定、error result discard、合法 AXI slave、reset 與 modulo arithmetic。未把本輪三個正常支援操作改列為 unsupported。

## 最終執行結果與觸發計數

**Integrated RTL 243/243 case IDs（27 TB）；multi-seed stress 108/108 jobs；V3 CDC 36/36 jobs；unit tests 31/31；Python syntax 43 files；lint/structural 8/8；whitespace PASS。** CI 中的 default-seed jobs 與 stress/CDC 有重疊，數字分開列，不相加為 unique cases。每次 transaction 與 simulator process 均有 timeout。

| 版本 | seed / MAC period(ns) | full / FIFO stalls | non-head completions | RRESP faults | AW先 / W先 | B / R stalls |
|---|---|---:|---:|---:|---:|---:|
| V1 | 1 / 7.5 | — / — | — | — | 95 / 69 | 784 / 108 |
| V1 | 19 / 30 | — / — | — | — | 87 / 74 | 899 / 108 |
| V1 | 2345 / 3.5 | — / — | — | — | 91 / 60 | 867 / 108 |
| V2 | 1 / 7.5 | 0 / 0 | 0 | 8 | 25 / 18 | 461 / 270 |
| V2 | 19 / 30 | 107 / 92 | 0 | 8 | 29 / 12 | 487 / 252 |
| V2 | 2345 / 3.5 | 0 / 0 | 0 | 8 | 21 / 21 | 441 / 280 |
| V3 | 1 / 7.5 | 276 / 275 | 87 | 8 | 23 / 17 | 461 / 272 |
| V3 | 19 / 30 | 3749 / 3711 | 92 | 8 | 23 / 16 | 449 / 279 |
| V3 | 2345 / 3.5 | 0 / 0 | 85 | 8 | 22 / 20 | 442 / 280 |

V1 每 seed accepted/completed=12/12、AW/W/B=180/180/180、AR/R=36/36、partial operands=96、zero operands=48；V2/V3 每 seed accepted/completed=12/12、AW/W/B=72/72/72、AR/R=48/48。DMA 每 run 共 1826 R beats、913 FIFO writes/reads，逐 job checked；ID reallocation 112（V2）／79（V3），不把 V2 當 OOO。每 seed 的完整 AR/R、last、FIFO wraps 與 counters 位於 raw logs。

Lint 警告保留：既有 EOFNEWLINE；V1 4-bit／V2 16-bit index 用於較小 arrays（qualified 範圍內有界）；burst stride／occupancy 擴展；unused bits；混合同步／非同步 reset。無 latch／multiple-driver finding，reset 警告不等於 RDC 已關閉。

## 綁定、freeze 與 blocker

7 個既有 fresh reduced formal tasks 的實際遠端 proof source/config 六份 hashes、sealed results 與 status 重新核對後重用；`mac_dma.v`、FIFO、ROB engine 均未變。未重跑 formal，不宣稱 wrapper/full-top equivalence。

`mac_dma_rob.v` 改變使 **qualified6 與其 7＋2 zero-delay mapped GLS 對最終 V3 的綁定失效**。未重用它們當新版 PASS，未轉移舊 PPA／timing。原 qualified6 artifacts、frozen source manifest、historical physical pins/templates/acceptance/formal 與 Gauss tar 保持原 bytes。既有 integrity check 現在正確 FAIL：`Publication does not implement qualified V3 source: rtl/mac_dma_rob.v`；沒有 exemption 或 bypass。

**具體 blocker：需以最終 freeze 來源重新執行 revision-bound pre-layout synthesis／electrical／CDC／constraint／STA／mapped GLS qualification，再重建 checkpoint package。** 本輪 RTL-only、不做 physical experiments，Gauss 交接仍暫停，因此未執行此項；沒有新的 synthesis／P&R／GLS PASS 宣稱。

[evidence/rtl-closure-freeze.json](evidence/rtl-closure-freeze.json) 凍結最終 RTL 與 43 個 functional input hashes。後續 README／P&R 工作不得修改功能 RTL；任何改動須重新開啟 closure 與受影響 evidence binding。

| 最終 RTL | SHA-256 |
|---|---|
| `rtl/axi_read_engine_rob.v` | `b5234cde4eaa4d7b0579fbc14b9f55a3216f20727c70c5d6302879f8a757d3dc` |
| `rtl/mac_accel.v` | `db26c08585406cb464b2eedc57fa04de576115861710a3712f8820deb6ebf709` |
| `rtl/mac_accel_axi.v` | `09fd46cd28cb44c5e84e2649d243c8fe7e887cd7a468996aa65df05478697d88` |
| `rtl/mac_accel_dma_rob_top.v` | `0f02ea10217a1138eab7f24c468acde9bd352de156e23db7df4bf383ce92e9a1` |
| `rtl/mac_accel_dma_top.v` | `1a3e48f541c8cdcc014f157b280be6da1756fd58542936b85d0c4ab141363b6d` |
| `rtl/mac_dma.v` | `eac52487e2fb5f053ee0278fb26576a2468bc57c7a8049a6c194ebc9690e0660` |
| `rtl/mac_dma_rob.v` | `18e3202c471b317f04883fa21d7de16d7e32768fe904fafd747c8355efacc61d` |
| `rtl/mac_fifo_async.v` | `cfeb105db0846f9ffc4072a2f20ffa5a79a13f6502454c77d8f0aeb3fc82c23e` |
| `rtl/mac_pe.v` | `a33d7b864149cf7eb58371199d6956c04bfb535b909c859273f78dfc2c2d5c39` |

重現：`python3 scripts/run_ci_rtl.py`、`python3 scripts/run_rtl_closure_stress.py`、`python3 sim/run_cdc_regression.py --simulator iverilog`、`python3 -m unittest discover -s tests -v`。此次未跑 VCS。

Raw logs、pre-fix snapshots、exact TB hashes、stress counters、CDC source binding 與 preservation：`/private/tmp/vector-mac-rtl-closure-cwbcqu6n`。本輪 diff 為該目錄的 `rtl-closure-review.diff`；與基準的完整候選 diff 為 `full-candidate-review.diff`。原 worktrees／歷史 artifacts／交接包保全結果見 `final-preservation.json`。本次至此停止。
