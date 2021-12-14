# Discrete Fully-Probabilistic Design (D-FPD)

## Introduction and related publications

D-FPD is a numerical/discrete version of an algorithm from [1]: this latter algorithm can tackle both discrete and continuous control problems by finding an analytical solution. D-FPD instead finds a purely numerical solution. As the original algorithm, the numerical/discrete implementation of D-FPD can be used to compute policies from examples for constrained, possibly stochastic/nonlinear, systems.

A detailed description of the D-FPD algorithm is available in the [accompanying paper](paper.pdf):

E. Ferrentino, P. Chiacchio, G. Russo, "The discrete fully probabilistic design algorithm: a tool to design control policies from examples". 2021.

## The code at a glance

This MATLAB repo is composed of two classes of scripts:

* **Proof-of-concept**: set of scripts to demonstrate how D-FPD works and compare it with the continuous counterpart
* **Inverted pendulum example**: set of scripts to demonstrate the effectiveness of D-FPD in generating a data-driven control policy on an inverted pendulum

## Executing proof-of-concept scripts

Open up MATLAB and make the repo main folder the MATLAB current directory. To run the proof-of-concept (D-FPD on a made-up example), run

```matlab
demo_discrete_fpd_base
```

Brief description of the other scripts:

* `demo_continuous_fpd` provides a continuous implementation of FPD, meaning that state/input domains are continuous;
* `discrete_continuous_comparison` performs a comparison between discrete and continuous FPD using data files generated from the scripts above. The user user might need to configure some parameters in the script before executing it.

## Executing the pendulum example

The pendulum example is divided into four phases

* Data generation
* Probabilistic model generation
* D-FPD optimization
* Probabilistic policy validation

Most of the scripts for this use case are contained in the `example` folder.

### Data generation

Data are generated with a model-based controller. The controller is given a time-varying trajectory reference bringing the pendulums to the unstable equilibrum state with randomized time parametrizations.

The actuated pendulum is made noisy through the introduction of a Gaussian noise acting at acceleration level.

Generate trajectories by running the script

```matlab
demo_noisy_pendulum_data_generation
```

A data file is generated in the `data` folder. If you do not want to re-generate data files, you can use those already available in this repo.

### Probabilistic model generation

The data files above can be used to generate a probabilistic model:

```matlab
demo_probabilistic_model_generation
```

The script generates a data file containing the state evolution models and the reference's randomized control law. The output data file will also contain information about the discretization of states and input. If you do not want to re-generate this data file, you can use that already available in this repo.

### D-FPD optimization

The probabilistic model can be used to generate an optimal control policy throguh D-FPD:

```matlab
demo_dfpd_2states_1input
```

Being application independent, the script above is located in the top folder of the repo.

The script generates the randomized control law for the target system in a data file in the `results` folder. If you do not want to re-generate this data file, you can use that already available in this repo. You can analyze these results by launching the script

```matlab
dfpd_2states_1input_results_analysis
```

### Probabilistic policy validation

The control policy can be loaded in a data-driven controller acting on the target system through the script

```matlab
demo_noisy_pendulum_validation
```

The script above will also launch the simulation showing the pendulum evolution subject to the probabilistic control policy.

## Authors and contributors

* Enrico Ferrentino (author)

## References

[1] Davide Gagliardi and Giovanni Russo. On a probabilistic approach to synthesize control policies from example datasets. _Automatica_, (in press), 2021. URL: [https://arxiv.org/pdf/2005.11191.pdf](https://arxiv.org/pdf/2005.11191.pdf).
