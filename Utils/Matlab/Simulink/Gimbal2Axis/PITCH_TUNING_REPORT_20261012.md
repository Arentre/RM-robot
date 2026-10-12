# Pitch tuning report (2026-10-12)

## Hardware state

- COM7 responded with the new `pfilter` field. The final state was STOP (`en=0`, `serial=0`, `stop=1`).
- Final runtime tuning was restored to the original Pitch values: LADRC `10,500,8,30,0.3`, friction `0.020,0.010,0.3,5,0.12`, gravity disabled, output filter `0 Hz`.
- Yaw parameters were not changed.

## Tests

| Test | Result |
|---|---|
| No filter, offsets `[5,-5,10,0]` | 3/4 phases passed. The reverse ~9.55 deg phase was marginally slow: settling `Inf`, tail error95 `0.600 deg`, rate RMS `2.25 deg/s`. |
| Pitch output filter 10 Hz | Two trials aborted in the first ~5 deg phase at measured rate `150.1` and `186.7 deg/s` while motor output was only `0.04`; not accepted as a stabilizing change. |
| No filter repeat, offsets `[5,0]` | 5 deg phase passed; peak rate `24.4 deg/s`, tail error95 `0.491 deg`. Return phase was marginal (`0.552 deg` error95, settling `2.29 s`). |
| Bidirectional 5 deg sweep, +/-15 deg | 11 valid angle pairs. Gravity fit amplitude `0.0511`, phase `-9.46 deg`, but cross-validation RMSE `0.00820` exceeded the `0.0025` acceptance limit. Gravity rejected. Moving-friction median was `0.0148`. |
| Second sweep around 15 deg absolute Pitch | Only 4 independent pairs were available, so that pose was rejected alone. The combined two-pose fit had amplitude `0.0324`, phase `-13.08 deg`, CV RMSE `0.00892`; gravity remained rejected. |
| Coulomb candidate `0.015` | Rejected: first two phases failed; reverse phase settling `3.994 s`, tail error95 `0.656 deg`. |

## Decision

Do not enable a gravity feedforward default and do not change the stored friction default yet. The current data shows a useful approximate moving-friction magnitude, but it is not enough to guarantee stability across direction and pose. The combined two-pose sweep is still not a defensible gravity model; inspect pose-dependent hysteresis and the feedback chain before increasing compensation gains.

Raw data and metric CSV files are in this directory with prefixes `pitch_hil_` and timestamps `20261012`.
