# Inverted Cart Pendulum – RL Swing-Up on STM32

Swing-up and balance of an inverted pendulum on a stepper-driven cart, using a
SAC policy trained in MuJoCo and deployed on an STM32F446RE with X-CUBE-AI.

## Repository layout

- **`cart_pendolo_velocita_rl/`** – Python side: MuJoCo/Gymnasium environment,
  SAC training (Stable-Baselines3), simulation, hardware-in-the-loop tests and
  firmware export. All parameters live in `config.toml`.
- **`Controllo_SwingUp_Finale/`** – STM32CubeIDE firmware: encoder reading,
  state estimator (`stima.c`), network inference (X-CUBE-AI) and stepper control.

## Quick start

```bash
cd cart_pendolo_velocita_rl
pip install -r requirements.txt

python addestra.py           # train a SAC agent
python simula.py             # run the trained policy in the MuJoCo viewer
python esporta_firmware.py   # export the network and config to the firmware
python hil.py                # hardware-in-the-loop test (firmware built with HIL_MODE 1)
```

Then build `Controllo_SwingUp_Finale` in STM32CubeIDE and flash it on the board.
Check the scripts for its options.

## Results

A technical report detailing the project's theoretical background, system architecture, algorithm design, and experimental results is available [here](report.pdf).
A video demonstration of the test on real plant (lab test) is available [here](https://politecnicobari-my.sharepoint.com/personal/s_febbraro_studenti_poliba_it/_layouts/15/stream.aspx?id=%2Fpersonal%2Fs%5Ffebbraro%5Fstudenti%5Fpoliba%5Fit%2FDocuments%2FEmbedded%20Control%2FInverted%5FPendulum%5FProva%5FBanco%2Emp4&share=cQqPm0YgMdoFQp2m177pBnoBEgUC22ZDDLI8M70crHVRaH1UCA&referrer=StreamWebApp%2EWeb&referrerScenario=AddressBarCopied%2Eview%2Ec39388fe%2D8c06%2D491d%2Dbd0e%2D35391cc08f9f).
