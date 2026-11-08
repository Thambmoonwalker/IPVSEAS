# IPVSEAS Part 1: Simulation and Test Plan (Lenovo Legion, RTX 5050, Windows)

Follow the phases in order. Each phase ends with a **✅ Pass if** check. Don't move on until it passes; if it fails, keep the exact output and the command you ran.

Commands are for **PowerShell**, run from the repository root (`IPVSEAS\`).

---

## Where Part 1 stands

| Done (tested in the cloud, synthetic data only) | Still to do (on your laptop) |
|---|---|
| C++ core: LiDAR, radar, camera fusion, tracking, prediction, risk, warnings | Build and run on Windows (MSVC) |
| 700/700 synthetic scenario runs | Verify the CARLA recorder on a live server |
| Recorder, dataset generator, replay, evaluation, summary | Tune on real CARLA data |
| | Cut-in scenario in CARLA |
| | Your own detection-only baseline |
| | V1 campaign, results, demo video |
| | Understanding every module well enough to explain it |

---

## Phase 0: Software setup (½ day)

**0.1 Put the code in your repo**
1. Extract `IPVSEAS_part1.zip` over your local `IPVSEAS` folder.
2. Commit and push:
   ```powershell
   git add -A
   git commit -m "Part 1: core, radar, dataset generator"
   git push
   ```

**0.2 Two Python environments** (CARLA's client may not support your Python 3.12, and YOLO needs PyTorch):

| Environment | Used for | Contents |
|---|---|---|
| `cv_env` (existing, Python 3.12) | YOLO, processing, evaluation, plots | PyTorch (CUDA), `pip install -r requirements.txt` |
| `carla_env` (new) | recording in CARLA | `carla`, numpy, opencv-python, pyyaml |

```powershell
# in cv_env
pip install -r requirements.txt      # do NOT reinstall torch
python -c "import torch, ultralytics; print(torch.cuda.is_available())"     # must print True
```

If PowerShell refuses to activate an environment: `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`.

**✅ Pass if** `cv_env` imports torch (CUDA True) and ultralytics.

---

## Phase 1: Build and unit tests (½ day)

**1.1 Local CMake preset** (uses your existing OpenCV and Eigen):
```powershell
copy CMakeUserPresets.json.example CMakeUserPresets.json
# edit CMakeUserPresets.json if your paths differ from C:/opencv/opencv/build and C:/libs/eigen-3.4.0
```

**1.2 Build and test** (in a normal PowerShell; CMake finds Visual Studio 2022 itself):
```powershell
cmake --preset local
cmake --build --preset local
ctest --test-dir build/local -C Release --output-on-failure
```

**1.3 Acceptance tables**
```powershell
.\build\local\part_1_road_perception\Release\part1_scenarios.exe --seeds 50
.\build\local\part_1_road_perception\Release\part1_scenarios.exe --seeds 50 --no-radar
```

**✅ Pass if**
- `ctest`: 2/2 tests passed.
- With radar: **700/700**, "0 scenario(s) with failures".
- `--no-radar`: only **S7_stationary_truck_120kmh** fails. That's expected; it proves radar matters.

If the build fails or the numbers differ, save the full output. MSVC can warn about things GCC accepted; usually these are one-line fixes.

---

## Phase 2: Tool chain without CARLA (1 hour)

```powershell
python simulation\carla\generate_dataset.py --config experiments\configs\synthetic_test.yaml --backend synthetic --process
python experiments\summarize_dataset.py datasets\runs\synthetic_test
python simulation\carla\view_run.py datasets\runs\synthetic_test\stationary_vehicle_120kmh_clear_none_truckdistance150_s01 --frame 20
python experiments\evaluate_run.py datasets\runs\synthetic_test\stationary_vehicle_120kmh_clear_none_truckdistance150_s01 --plot
```

**✅ Pass if** the summary table shows (lead_med):

| speed | full | no_radar | lidar_only |
|---|---|---|---|
| 60 | 2.50 | 2.50 | 2.50 |
| 90 | 3.20 | 3.15 | 3.20 |
| 120 | **3.60** | **2.30** | 3.60 |

You should also see a bird's-eye view with a red track on the green truck box, and an `evaluation.png` where the predicted TTC follows the true time to impact.

---

## Phase 3: YOLO on real video (½ day)

```powershell
python part_1_road_perception\camera\detection\yolo_detect.py --video datasets\sample_videos\road_video.mp4 --show
python part_1_road_perception\camera\detection\yolo_detect.py --video datasets\sample_videos\road_video.mp4 --imgsz 640
python part_1_road_perception\camera\detection\yolo_detect.py --video <indian_dashcam_clip.mp4> --show --csv yolo_dashcam.csv
```

Record in your log (template at the end):
- ms/frame at imgsz 640 and 960 (printed at the end);
- missed pedestrians, animals and two-wheelers (watch with `--show`);
- what auto-rickshaws, tractors and handcarts are labelled as (COCO has no class for them).

**✅ Pass if** 960 px runs at ≤ 20 ms/frame, or you've decided to use 640 px and noted the accuracy trade-off.

---

## Phase 4: CARLA installation (½ day)

1. Download **CARLA 0.9.16 for Windows** from the CARLA releases page and extract it to a short path, e.g. `C:\CARLA_0.9.16` (long paths cause errors).
2. Start the server and wait until the map appears:
   ```powershell
   C:\CARLA_0.9.16\CarlaUE4.exe -quality-level=Low
   ```
3. Create `carla_env` with a Python version that has a CARLA 0.9.16 wheel. Look in `C:\CARLA_0.9.16\PythonAPI\carla\dist\`; the file name shows the Python version, e.g. `cp310`/`cp311`/`cp312`.
   ```powershell
   py -3.10 -m venv carla_env           # use the version the wheel was built for
   .\carla_env\Scripts\Activate.ps1
   pip install C:\CARLA_0.9.16\PythonAPI\carla\dist\<wheel file>.whl   # or: pip install carla==0.9.16
   pip install numpy opencv-python pyyaml matplotlib
   python -c "import carla; c=carla.Client('localhost',2000); c.set_timeout(10); print(c.get_server_version())"
   ```

**✅ Pass if** the last command prints `0.9.16`.

Keep an eye on GPU memory in Task Manager. If CARLA uses most of the 8 GB, don't run YOLO at the same time. The workflow below records first and processes afterwards anyway.

---

## Phase 5: First recordings and the three sanity checks (1–2 days)

**5.1 Quick trial** (`carla_env`, CARLA running):
```powershell
python simulation\carla\generate_dataset.py --config experiments\configs\quick_trial.yaml --dry-run
python simulation\carla\generate_dataset.py --config experiments\configs\quick_trial.yaml
```
Note how long each run takes: **recording rate = frames ÷ seconds**. You'll pass it as `--sim-fps` later.

**5.2 Check 1: axes and alignment** (most important)
```powershell
python simulation\carla\view_run.py datasets\runs\quick_trial\lead_brake_72kmh_clear_none_s01 --frame 60
```
- A vehicle on your **left** must appear at **+y** (upper half of the plot).
- Black LiDAR points and purple radar crosses must sit **on** the green ground-truth boxes.
- If everything is mirrored left/right, the y-conversion in `record_run.py` is wrong. Fix it before anything else.

**5.3 Check 2: radar velocity sign**
```powershell
python simulation\carla\view_run.py datasets\runs\quick_trial\lead_brake_72kmh_clear_none_s01 --radar-check
```
If it says `use radar_velocity_sign = -1`:
1. Set `RADAR_VELOCITY_SIGN = -1.0` in `record_run.py`.
2. Delete the quick_trial runs and record them again (`--force`).

**5.4 Check 3: radar density**
```powershell
python -c "import numpy as np; d=np.fromfile(r'datasets\runs\quick_trial\stationary_vehicle_72kmh_clear_none_s01\radar\000040.bin',dtype=np.float32).reshape(-1,5); print(len(d), 'radar points'); print(np.round(d[np.abs(d[:,1])<0.05][:,0],1))"
```
The numbers in the second line are ranges of returns straight ahead; the truck should be among them. If distant objects get fewer than 2 points, raise `points_per_second` in `RADAR` (record_run.py) and re-record.

**5.5 Process and look at the results** (switch to `cv_env`; CARLA can be closed):
```powershell
python simulation\carla\generate_dataset.py --config experiments\configs\quick_trial.yaml --process-only
python experiments\summarize_dataset.py datasets\runs\quick_trial
python experiments\evaluate_run.py datasets\runs\quick_trial\lead_brake_72kmh_clear_none_s01 --plot
```

**✅ Pass if**
- `lead_brake` and `stationary_vehicle` warned before the collision (not "missed").
- `free_drive` has no false-alarm frames.
- Tracking recall is ≥ 90 %.

If not, go to Phase 6.

**Common problems**

| Symptom | Likely cause / fix |
|---|---|
| `queue.Empty` / timeout while recording | the server is too slow: lower quality, fewer NPCs, or raise the `timeout` values in `record_run.py` |
| "could not spawn the scenario actor" | the spawn point is near a junction; record again with another `--seed` |
| ego drives into a junction or turns off the highway | use Town04 and a different seed; check `run_config.json` |
| everything mirrored in the BEV | y-axis conversion (Check 1) |
| the static world shows huge radar velocities | radar sign (Check 2) |
| CARLA crashes when spawning traffic | use `--traffic light` first |

---

## Phase 6: Tuning on CARLA data (only if Phase 5 fails; 2–4 days)

The rule: **change one parameter at a time, rerun `ctest` after each change** (the synthetic tests are your regression safety net), and write every change in the log.

To find the cause of a problem:
1. Open `part1_output.csv` and find the first frame where `level` is 2 (WARNING) when it shouldn't be, or stays 0 when it should be 2.
2. Look at that frame with `view_run.py --frame N`. Is the track where the object is? Is there a track where nothing is?

| Problem | Where to look | Parameter |
|---|---|---|
| objects not tracked (low recall) | `lidar/obstacle_detector.hpp` (`LidarConfig`) | `min_points`, `min_height`, `cell_size`, `roi_x_max` |
| ground or kerbs detected as obstacles | same | `ground_dist`, `min_height` |
| one object becomes several tracks | `LidarConfig::cell_size`, radar `cluster_eps` | larger values |
| false alarm from a parked car or guardrail | `RiskConfig::safety_margin`, `PredictionConfig::lateral_sigma_gate` | check the track's lateral velocity first |
| warning too late | maturity: `TrackerConfig::mature_vel_sigma`, `confirm_hits` | check *when* the track became mature |
| camera class missing | `FusionConfig::min_iou`, YOLO `--conf` | check the projection with `view_run.py` |

**✅ Pass if** Phase 5's checks pass and `ctest` is still green.

---

## Phase 7: V1 campaign (≈ 1 week incl. processing)

1. **Check the cost** (needs ~22 GB). To store data on another drive, set `output:` in the YAML to an absolute path, e.g. `D:/ipvseas_runs/v1_campaign`.
   ```powershell
   python simulation\carla\generate_dataset.py --config experiments\configs\v1_campaign.yaml --dry-run --sim-fps <your rate>
   ```
2. **Record** (`carla_env`; it can run overnight, and rerunning resumes):
   ```powershell
   python simulation\carla\generate_dataset.py --config experiments\configs\v1_campaign.yaml
   ```
3. **Process and summarise** (`cv_env`):
   ```powershell
   python simulation\carla\generate_dataset.py --config experiments\configs\v1_campaign.yaml --process-only
   python experiments\summarize_dataset.py datasets\runs\v1_campaign
   ```
4. **Compare** `summary.md` with the V1 acceptance criteria (spec section 6.2):

| Metric | Target |
|---|---|
| lead time, lead_brake / stationary_vehicle | ≥ 1.5 s, no misses |
| lead time, pedestrian_cross | ≥ 1.0 s |
| stationary_vehicle at 120 km/h (full) | ≥ 2.8 s |
| free_drive false-alarm runs | 0 |
| tracking recall | ≥ 90 % |

5. Copy `summary.md`, `summary.csv` and the best plots into `results/` (Markdown, CSV and PNG files there are tracked by git) and commit.

**✅ Pass if** every row meets its target, or the misses are understood and explained in the log.

---

## Phase 8: Comparisons for your report (2–3 days)

1. **Sensor ablation** (already in the summary): `full` vs `no_radar` vs `lidar_only`. Expected pattern: radar matters most for stationary_vehicle at 100–120 km/h; the camera matters for classes (pedestrian/animal behaviour).
2. **Your own baseline (write it yourself).** A simple rule like "warn when an in-lane object is closer than 2 seconds of travel" (distance < 2 s × ego speed), computed from `tracks.csv` and `ego.csv` in a small Python script. Compare its lead times and false alarms with IPVSEAS on the same runs. This is the single most important comparison: it shows what *prediction* adds over *detection*.
3. **Lead time vs speed plot** for stationary_vehicle, against the theoretical curve t_react + v/(2·a) (spec section 4.10).

---

## Phase 9 (optional): robustness and unexpected events

```powershell
python simulation\carla\generate_dataset.py --config experiments\configs\weather_and_surprises.yaml --dry-run
```
Record and process it the same way. Remember that CARLA weather mainly degrades the **camera**, so read the results as camera robustness until the Part 2 LiDAR/radar weather models exist.

---

## Daily log template

Copy into `results/test_log.md` and fill in as you go:

```markdown
## <date> - Phase <n>
- Command(s):
- Result / numbers:
- Pass?  yes / no
- Problem and cause (if any):
- Change made (file, parameter, old -> new):
- ctest after change:  pass / fail
- Next step:
```
