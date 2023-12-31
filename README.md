# FastUAV Refinement of the structural module 

FAST-UAV @felixpollet
======================

FAST-UAV is a solution to perform optimal drone design with a multi-disciplinary approach.

Based on the [FAST-OAD](https://github.com/fast-aircraft-design/FAST-OAD) framework, it allows to easily switch between models to address different types of configurations. 

Currently, FAST-UAV is bundled with analytical models for multirotor, fixed wing and hybrid-VTOL UAVs.

Install
-------
1. Download the zip file from Github and unzip it. Alternatively, you can clone the repository with the `git clone` command.

2. Open an Anaconda Prompt terminal and navigate to the recently unziped folder.

> ``` {.bash}
> cd path/to/your/fastuav/folder
> ```

3. If you haven't done so before, install [Poetry](https://python-poetry.org/docs/) using [pipx](https://pypa.github.io/pipx/):
> ``` {.bash}
> python -m pip install --user pipx
> pipx install poetry==1.2.0
> ```

4. Create a new conda environment and activate it by running:

> ``` {.bash}
> conda create -n fastuav python=3.8
> conda activate fastuav
> ```

5. Install the required dependencies with [Poetry](https://python-poetry.org/docs/):
> ``` {.bash}
> poetry install
> ```

Run
-------
Once FAST-UAV is installed, you can access the notebooks with Jupyter Lab:
> ``` {.bash}
> jupyter lab
> ```
If this command is not found by Python, you can install Jupyter Lab with ```conda install -c conda-forge jupyterlab```.

Jupyter lab will open automatically in your browser. Then, navigate to the `src/fastuav/notebooks` directory and start with the first notebook.

**Note**: Every time you will be working with FAST-UAV, you must first activate the conda environment with the `conda activate fastuav` command before opening the jupyter notebooks.


Publications
------------
> M. Budinger, A. Reysset, A. Ochotorena, and S. Delbecq, ‘Scaling laws and similarity models for the preliminary design of multirotor drones’, Aerospace Science and Technology, vol. 98, p. 105658, Mar. 2020, doi: 10.1016/j.ast.2019.105658.

> S. Delbecq, M. Budinger, A. Ochotorena, A. Reysset, and F. Defay, ‘Efficient sizing and optimization of multirotor drones based on scaling laws and similarity models’, Aerospace Science and Technology, vol. 102, p. 105873, Jul. 2020, doi: 10.1016/j.ast.2020.105873.

> F. Pollet, S. Delbecq, M. Budinger, and J.-M. Moschetta, ‘Design optimization of multirotor drones in cruise’, Sep. 2021.

> S. Delbecq, M. Budinger, C. Coic, and N. Bartoli, ‘Trajectory and design optimization of multirotor drones with system simulation’, VIRTUAL EVENT, United States, Jan. 2021. doi: 10.2514/6.2021-0211.

> J. Liscouet, F. Pollet, J. Jézégou, M. Budinger, S. Delbecq, and J.-M. Moschetta. “A Methodology to Integrate Reliability into the Conceptual Design of Safety-Critical Multirotor Unmanned Aerial Vehicles.” Aerospace Science and Technology, June 1, 2022, 107681. https://doi.org/10.1016/j.ast.2022.107681.

> F. Pollet, S. Delbecq, M. Budinger, J.-M. Moschetta, and J. Liscouët. “A Common Framework for the Design Optimization of Fixed-Wing, Multicopter and VTOL UAV Configurations.” In 33rd Congress of the International Council of the Aeronautical Sciences. Stockholm, Sweden, 2022. https://hal.archives-ouvertes.fr/hal-03832115.

> F. Pollet, M. Budinger, S. Delbecq, J. -M. Moschetta, and J. Liscouët. “Quantifying and Mitigating Uncertainties in Design Optimization Including Off-the-Shelf Components: Application to an Electric Multirotor UAV.” Aerospace Science and Technology, May 1, 2023, 108179. https://doi.org/10.1016/j.ast.2023.108179.

> [DroneApp](https://github.com/SizingLab/droneapp-legacy) sizing tool

