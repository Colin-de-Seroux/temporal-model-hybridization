# Before using tgnn

Placed in `/tgnn` :

1. Create `.venv`

2. Activate it :

```bash
.\.venv\Scripts\activate
```

3. Run `nvcc --version` and get the good cuda version on pytorch for your version (you can use 11.8 with cuda 12.6) :

```bash
nvcc --version
```

[Pytorch website](https://pytorch.org/get-started/locally/)

4. Install dependencies :

a. Manually :

```bash
pip install torch==2.7.0 torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

```bash
pip install torch-scatter torch-sparse -f https://data.pyg.org/whl/torch-2.7.0+cu118.html
```

```bash
pip install -r requirements.txt
```

b. With `.sh` :

Setup.sh has all the dependancies to install.

```bash
./setup.sh
```

> [!Warning]
> If you have an error like this on windows:

```sh
error: Microsoft Visual C++ 14.0 or greater is required. Get it with "Microsoft C++ Build Tools": https://visualstudio.microsoft.com/visual-cpp-build-tools/
      [end of output]

  note: This error originates from a subprocess, and is likely not a problem with pip.
  ERROR: Failed building wheel for torch_sparse
  Running setup.py clean for torch_sparse
Failed to build torch_scatter torch_sparse
ERROR: Failed to build installable wheels for some pyproject.toml based projects (torch_scatter, torch_sparse)
```

1. Go download [Visual Studio Setup](https://visualstudio.microsoft.com/visual-cpp-build-tools/).

2. Click on `C++ build tools`.

3. Click on `Download`.

## Training

### Description

**Input :**

- csv with datas froms simulations save in `dataset/dataset.csv`

**Output :**

- save the model trained in `save/model.pth`
- `save/feature_scaler.pkl`
- `save/target_scaler.pkl`

**Model :**

`GConvGRU` : temporal prediction in ROS 2 system, where the structure and timing of actions (such as publish, subscribe, timer) evolve over time and can be reprsented as a dynamic graph.

**Graph nodes :**

Represent ROS 2 _actions_ or _triggers_: `Publisher`, `Subscriber`, `Timer` ...

**Graph edges :**

- **Topic connections** (e.g., a `Publisher` connected to a `Subscriber`) with _edge weight = abs(subscriber_execution_time - publisher_execution_time)_
- **Timer-based triggers** (e.g., a `Timer` connected to a `Publisher`) with _edge weight = abs(publisher_execution_time - timer_execution_time)_

The graph is **dynamic** over time, **new actions** may **appear/desappear** and **connections** may **change** (based on **active topics** or **triggered timers**).

**`DynamicGraphTemporalSignal`** : Each time snapshot int the dataset represents:

- The **graph** of active ROS 2 actions/triggers at time `t`
- The **edges** (topics/timers) and their weights
- Node **features** : all dataset columns except `ExecutionTime`
- **Target** values : execution time

### Launch tgnn_project

```bash
cd tgnn_project
python __main__.py
```

## Prediction

### Description

#### Flask API

**Input :**

- `save/model.pth`
- `save/feature_scaler.pkl`
- `save/target_scaler.pkl`

**Routes :**

- `/receive-ast` : receive in tabular format the informations of `.aml` configuration

### Launch tgnn_service

```bash
cd tgnn_service
python run.py
```
