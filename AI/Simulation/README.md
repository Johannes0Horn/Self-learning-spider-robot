# Installation Guide For Custom GYM Environment

### 1.  Open console
### 2.  Open folder
    - Navigate inside the console to: "*/bug/Simulation/gym-robert" without quotes.
    - The gym-robert folder contains a setup.py file.
    - Important note: The folders gym-robert and gym_robert are different!
### 3.  Install pybullet
    - Execute command: "pip install pybullet" without quotes.
### 4.  Install custom gym environment
    - Execute command: "pip install -e ." without quotes.
    - Important note: the . is part of the command!
    - The command depends on the current folder!
### 5. After successful installation
    - You can "import gym_robert" in your python file.
    - Important note: The folders gym-robert and gym_robert are different!
### 6. Create gym environment
    - "env = gym.make("robert-v0")"
### 7. See RunRobertRun.py for a small example
### 8. Why folders with - and _ ?
    - Official gym documentation and to annoy you! :D