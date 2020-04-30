# Scripts in MATLAB

## Directory organization
- `filters/`: implementation of LIEKF and EKF
- `helper/`: helper functions, including data loader, fake data generator, and
  function to convert lie algebra to cartesian
- `thirdparty/`: third party toolboxes we used, isolated in their own respective 
  folders.
    - `shadedErrorBar`: A library to generate shaded error margins. Package may
      be obtained [here](https://github.com/raacampbell/shadedErrorBar)
- `deprecated/`: old scripts we no longer use

## Driver scripts
- `fake_data_test_inekf.m`: runs inekf on fake data
- `fake_data_test_plot.m`: runs inekf and ekf on fake data, generate plots
  including predicted mean and stdev for each filter
- `zurich_test_inekf.m`: runs inekf on the zurich urban data
- `zurich_test_plot.m`: runs inekf and ekf on the zurich and generate
  trajectory plot for each filter



