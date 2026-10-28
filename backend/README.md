# Backend

Python scripts for recording sensor data, estimating positions, and evaluating experiments.

## Setup

For offline processing and analysis:

```bash
python3 -m venv /tmp/uwb-analysis-venv
source /tmp/uwb-analysis-venv/bin/activate
python -m pip install numpy pandas scipy matplotlib seaborn tabulate
```

For MQTT recording, also install `paho-mqtt`. The live viewers additionally need `pyqtgraph` and a Qt binding such as `PySide6`:

```bash
python -m pip install paho-mqtt pyqtgraph PySide6
```

The scripts use filenames relative to the current working directory. Run them from the folder containing the experiment data. Existing output files with the same names are overwritten.

## Data collection

Set the broker address and credentials in each MQTT script before starting it. The default topics are `uwb/data` and `imu/data`.

| Script | Purpose | Output |
| --- | --- | --- |
| [collection/mqtt_logger.py](collection/mqtt_logger.py) | Record UWB ranges and IMU samples received over MQTT. | `uwb_data_1.csv`, `imu_data_1.csv` |
| [collection/ground_truth.py](collection/ground_truth.py) | Detect stops from live IMU data, assign predefined stop positions, and interpolate movement between them. Set the path and stop parameters to match the experiment. | `mqtt_ground_truth.csv`, saved when recording is stopped with Ctrl+C |

The logger clears its two output CSVs at startup. Use a new recording directory for each experiment. Run the ground-truth recorder alongside the logger when both outputs are needed.

## Processing

| Script | Purpose | Input | Output |
| --- | --- | --- | --- |
| [processing/merge_imu_uwb.py](processing/merge_imu_uwb.py) | Assign each UWB range to the next IMU timestamp within 100 ms; leave other range cells empty. | `imu_data_1.csv`, `uwb_data_1.csv` | `merged_imu_uwb_data.csv` |
| [processing/run_ekf.py](processing/run_ekf.py) | Estimate 2D position from IMU data and UWB ranges, with stationary detection and zero-velocity updates. | `merged_imu_uwb_data.csv` | `ekf_results.csv` |
| [processing/run_trilateration.py](processing/run_trilateration.py) | Estimate 2D position using height-corrected UWB ranges and least squares. | `merged_imu_uwb_data.csv` | `trilat_results.csv` |
| [processing/run_imu_dead_reckoning.py](processing/run_imu_dead_reckoning.py) | Integrate orientation-corrected acceleration without UWB corrections. | `merged_imu_uwb_data.csv` | `imu_dead_reckoning.csv` |

Anchor coordinates, tag height, initial position, and filter parameters are defined in the scripts. The shared EKF starts at `(2.07, 0.70)` m; [exp2_2/run_ekf.py](../results/exp2_2/run_ekf.py) retains that experiment's different initial position. Check these settings before processing a new recording. [reference/static_position.txt](reference/static_position.txt) contains the original coordinate notes.

## Analysis

| Script | Purpose | Input | Output |
| --- | --- | --- | --- |
| [analysis/plot_errors.py](analysis/plot_errors.py) | Compare EKF and trilateration errors against interpolated ground truth; print statistics and plot error over time and the CDF. | `mqtt_ground_truth.csv`, `ekf_results.csv`, `trilat_results.csv` | `error_comparison.png` |
| [analysis/compare_errors_smoothness.py](analysis/compare_errors_smoothness.py) | Compare position error, path length, and a smoothness proxy based on mean acceleration magnitude. The script labels this proxy as jerk. | Same three CSVs as `plot_errors.py` | `smoothness_comparison_with_metrics.png` |
| [analysis/plot_stop_errors.py](analysis/plot_stop_errors.py) | Evaluate trilateration during stops using exact timestamp matches with ground truth. | `mqtt_ground_truth.csv`, `trilat_results.csv` | `stop_phase_error_boxplot.png`, `stop_phase_position_scatter.png`, `stop_phase_error_over_time.png` |
| [analysis/sample_rate_imu.py](analysis/sample_rate_imu.py) | Check IMU timestamp intervals, estimated packet gaps, and samples per second against 50 Hz. | `imu_data_1.csv` | Console statistics, `sample_rate_histogram.png` |
| [analysis/sample_rate_uwb.py](analysis/sample_rate_uwb.py) | Count UWB ranges per anchor within the ground-truth time window against an expected 5 Hz per anchor. | `uwb_data_1.csv`, `mqtt_ground_truth.csv` | `uwb_data_1_summary.csv`, console statistics |
| [analysis/plot_sample_rate_comparison.py](analysis/plot_sample_rate_comparison.py) | Compare per-anchor reception percentages across the three stored scenarios. | `uwb_sample_los.csv`, `uwb_sample_wlos.csv`, `uwb_sample_nlos.csv` in `results/sample_rate/` | `uwb_scenario_comparison_by_anchor.png` |

## Live viewers

| Script | Purpose | Output |
| --- | --- | --- |
| [visualization/position_plotter.py](visualization/position_plotter.py) | Display live UWB trilateration positions and IMU orientation from MQTT. | Qt window |
| [visualization/sensor_dashboard.py](visualization/sensor_dashboard.py) | Display live IMU components and UWB ranges from MQTT. | Qt window; the save button exports `imu_data_plot_<timestamp>.csv` and `uwb_data_plot_<timestamp>.csv` |

## Examples

From the repository root, plot an existing experiment:

```bash
cd results/exp2_3
python ../../backend/analysis/plot_errors.py
python ../../backend/analysis/compare_errors_smoothness.py
```

To recompute positions without modifying the stored experiment, copy its CSVs into a scratch directory. Run this block from the repository root:

```bash
repo_root="$PWD"
work_dir=$(mktemp -d)
cp results/exp1_1/*.csv "$work_dir/"
cd "$work_dir"
python "$repo_root/backend/processing/merge_imu_uwb.py"
python "$repo_root/backend/processing/run_trilateration.py"
python "$repo_root/backend/processing/run_ekf.py"
python "$repo_root/backend/analysis/plot_errors.py"
```

For plots without a display, prefix a plotting command with `MPLBACKEND=Agg`.
See the [results README](../results/README.md) for retained experiment variants and the stored file formats.
