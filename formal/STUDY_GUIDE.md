# Formal Verification — Study Guide (MAC DMA project)

A self-contained course to *understand* the formal in this repo and *talk about it*
in an interview. Worked through bottom-up; every concept points at the exact lines
in `../rtl/mac_dma.v` and `mac_dma_bp.sby`.

---

## Module 0 — The one-sentence frame

> "Simulation shows the design works for the stimuli I *thought of*; formal proves
> a property holds for **every** input the solver can construct, up to a bound (BMC)
> or for all time (k-induction)."

If you can only say one thing, say that. Everything below is detail behind it.

---

## Module 1 — assume vs assert (the whole game)

- `assert (P)` — the solver must **prove** P can never be false. Find one input
  trace that breaks P → counterexample (CEX).
- `assume (P)` — the solver is **told** to treat P as always true; it only explores
  inputs where P holds. Used to model the environment / constrain free inputs.

Mental model: **assumptions narrow the input space, assertions are what you owe.**
Over-assume and you "prove" something vacuously (constrained away the bug). The art
is asserting a lot while assuming as little as possible.

In this project:
- `assume (length == 16'd2)`  → environment constraint (keep the trace short).
- `assume (rst)` in step 0    → force a clean reset start.
- the four `assert`s          → what we owe: VALID-stability / no-drop.

Interview line: *"The skill in formal is keeping your assumptions honest — every
assume you add is a piece of the proof you're choosing not to check."*

---

## Module 2 — Free inputs & the adversarial solver

Any module input not driven and not assumed is **free**: the solver picks its value
every cycle to try to break an assertion. Here `stream_ready` (= `~fifo_full`) is
free, so the solver explores **every possible back-pressure pattern** a downstream
FIFO could ever produce — not the three or four you'd hand-write in a testbench.

That is *why* formal caught the back-pressure data-drop bug that simulation missed:
the bad pattern was a specific cycle-by-cycle ready toggling you wouldn't think to write.

Interview line: *"I let `stream_ready` float as a free input so the solver drives
back-pressure adversarially — that's the coverage a directed testbench can't give."*

---

## Module 3 — The $past idiom and f_past_valid

Temporal properties need "the value last cycle" → `$past(x)`. But at cycle 0 there is
no previous cycle, so `$past` is garbage there.

```verilog
reg f_past_valid = 1'b0;
always @(posedge clk) f_past_valid <= 1'b1;   // 0 on cycle 0, then 1 forever
```

Guard every `$past`-based assertion with `f_past_valid` so cycle 0 is skipped. This
is boilerplate in every formal testbench — know it cold.

---

## Module 4 — BMC (mode bmc)

Bounded Model Checking: unroll the design `depth` cycles from reset and ask the SMT
solver "does any input trace violate an assert within these N steps?"

- PASS = no violation **within the bound** (here 40 cycles).
- Cheap, and CEXs are short/readable → the debug workhorse.
- Limitation: says nothing about cycle N+1. Not an unbounded proof.

In `mac_dma_bp.sby`: `bmc: mode bmc` / `bmc: depth 40`. 40 covers a full
IDLE→read-A→read-B→DONE pass for length=2.

---

## Module 5 — k-induction (mode prove)

Two independent obligations, **both** must pass:

1. **Base case** — a BMC of length k: property holds for the first k cycles from reset.
2. **Induction step** — *assume* the property held for k consecutive cycles starting
   from an **arbitrary** state, *prove* it holds on cycle k+1.

If both close, the property holds for **all time** → a real unbounded proof.

The catch — and the best interview story here:
the induction step starts from an *arbitrary* register state, which may be
**unreachable** in the real design. The solver happily starts there and fabricates a
**spurious** counterexample (CEX). There are **two knobs** to kill spurious CEXs, and
in practice you usually need *both* — one alone is often not enough:
- **raise k** — require the property to hold for more consecutive cycles before the
  checked step; longer histories squeeze out junk start-states.
- **add an inductive invariant** — an `assert` of a true fact that also fences the
  start state into the reachable region.

In this project the invariant is:
```verilog
a_state_legal : assert (state <= S_DONE);   // FSM never in illegal 3'd6/3'd7
```
It does double duty: it's a true property *and* it helps fence the induction start
state. The core VALID-stability property is otherwise nearly structural, because
`M_AXI_RREADY` gates new data purely combinationally on `!stream_valid || stream_ready`
— independent of FSM reachability.

`prove: mode prove` / `prove: depth 12`  (12 = k). Both basecase and induction pass.

### What I actually saw stress-testing the induction step (do this, it cements it)
Temporarily comment out the invariant and/or shrink `prove: depth`, then `sby -f
mac_dma_bp.sby prove`:

| config | basecase | induction | fails on |
|---|---|---|---|
| no invariant, k=1  | pass | **FAIL** | `a_last_stable` (core property) |
| invariant, k=1     | pass | **FAIL** | `a_state_legal` (state = 3'd6/7) |
| invariant, k=12    | pass | **pass**  | — (the real config) |

The k=1 / no-invariant CEX starts from an unreachable state — `state=S_IDLE` yet
`stream_valid=1, stream_last=1` (after reset, idle ⇒ valid=0, so this never happens)
— then pulses `rst` to flip `stream_last` and break `a_last_stable`. Pure spurious CEX.
Takeaway: **the induction step closes only when k is large enough AND the invariants
are strong enough — the two knobs work together.** k=12 + the legal-state invariant
is what closes it here.

Interview line: *"I stress-tested the induction step by dropping the invariant and
lowering k — watched it manufacture a spurious counterexample from an unreachable
state, then closed it with the right combination of induction depth and an FSM
legal-state invariant."*

---

## Module 6 — The property we actually proved

AXI-Stream handshake law: **once VALID is high it must stay high with payload frozen
until the consumer asserts READY; VALID must never depend on READY.**

```verilog
always @(posedge clk)
  if (f_past_valid && !rst && !$past(rst)
      && $past(stream_valid) && !$past(stream_ready)) begin
      a_valid_stable : assert (stream_valid);                 // didn't drop
      a_dataA_stable : assert (stream_a   == $past(stream_a)); // didn't change
      a_dataB_stable : assert (stream_b   == $past(stream_b));
      a_last_stable  : assert (stream_last == $past(stream_last));
  end
```

Read it as: *"if last cycle I was offering a beat and you didn't take it, this cycle
the exact same beat is still on the wires."* Violating it = a dropped/overwritten
element = the back-pressure data-drop bug, now provably impossible.

What makes it hold in the RTL:
- drain clears VALID **only** on accept: `if (stream_valid && stream_ready) stream_valid<=0;`
- `M_AXI_RREADY = (state==S_R_B) ? (!stream_valid || stream_ready) : ...` — no new
  beat is fetched while an un-accepted one is held → payload can't be overwritten.

---

## Module 7 — Running it

```bash
source ~/oss-cad-suite/environment
cd formal
sby -f mac_dma_bp.sby            # both tasks
sby -f mac_dma_bp.sby bmc        # just BMC (fast debug loop)
sby -f mac_dma_bp.sby prove      # just the unbounded proof
```
PASS = property holds, no trace produced. FAIL = a `.vcd` CEX appears in the task's
`engine_0/` dir → open in GTKWave and read the solver's shortest path to the bug.

---

## Interview: the 30-second pitch

> "I formally verified the DMA's stream output with SymbiYosys. I asserted the
> AXI-Stream no-drop law — VALID held with payload frozen until READY — and let the
> downstream ready signal float as a free input so the solver drove back-pressure
> adversarially. BMC caught a real data-drop bug a directed testbench missed. Then I
> upgraded to k-induction for an unbounded proof; the induction step needed an FSM
> 'legal-state' invariant to avoid spurious CEXs from unreachable states. Base case
> and induction both close."

## Interview: likely follow-ups → answers

- **"BMC or unbounded?"** → "Both. BMC for fast debug, k-induction (`mode prove`) for
  the unbounded guarantee. Base case + induction both pass."
- **"How do you know you didn't over-constrain?"** → "Only two assumes: a clean reset
  in step 0, and length pinned to 2 to bound the trace. The adversarial signal,
  `stream_ready`, is left fully free — that's where bugs hide, and it's unconstrained."
- **"Why did 1-induction / a naive prove fail?"** → "Induction starts from an arbitrary,
  possibly unreachable state — e.g. idle with stream_valid=1, or illegal `state`
  encodings 3'd6/3'd7 — and fabricates a spurious CEX. You close it with two knobs
  together: raise k *and* add an inductive invariant (`assert(state <= S_DONE)`). One
  alone wasn't enough; k=12 plus the invariant closes it."
- **"What property exactly?"** → Module 6, recite the four asserts in words.
- **"Liveness too?"** → "These are safety properties (nothing bad happens). I didn't
  prove liveness (eventually-accepts); that'd need fairness assumptions on READY."
- **"Limitation of your proof?"** → "It's parameterized at length=2 and proves the
  handshake invariant, not full functional correctness of the MAC result — that's
  covered by simulation. Honest scoping."

## If asked to go further (shows depth)

- Free `length` instead of pinning it (cover all transfer sizes).
- Add AR-channel stability asserts (ARVALID held until ARREADY; ARADDR stable).
- Prove `dma_err` is sticky once an RRESP error is seen.
- Add a cover() for "a full transfer completes" to confirm the proof isn't vacuous.
