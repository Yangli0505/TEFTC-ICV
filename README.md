# TEFTC-ICV

Official MATLAB implementation of the paper:

**Transmission-Efficient Fault-Tolerant Control for Intelligent and Connected Vehicles With Input Quantization and Event-Triggered Mechanism**

Published in **IEEE Transactions on Intelligent Transportation Systems (T-ITS), 2025**.

---

## Overview

This repository provides the MATLAB implementation of the transmission-efficient fault-tolerant control method proposed in our paper.

The method is developed for intelligent and connected vehicles under practical communication and actuator constraints. It jointly considers:

* **Actuator faults**
* **Input quantization**
* **Event-triggered communication**
* **Vehicle stability and tracking control**

The proposed framework aims to maintain reliable vehicle control performance while reducing unnecessary communication transmissions.

---

## Method

The proposed control framework integrates fault-tolerant control with input quantization and an event-triggered transmission mechanism.

The main features include:

1. **Fault-tolerant control under actuator faults**
   The controller is designed to maintain vehicle stability and tracking performance in the presence of actuator faults.

2. **Quantized control input**
   Input quantization is explicitly considered to account for limited communication bandwidth and digital transmission constraints.

3. **Event-triggered transmission mechanism**
   Control information is transmitted only when the prescribed triggering condition is satisfied, thereby reducing unnecessary communication.

4. **Transmission-efficient vehicle control**
   The proposed strategy achieves a balance between control performance, fault tolerance, and communication efficiency.

---

## Code

The MATLAB implementation is provided in:

[`tits_code.m`](https://github.com/Yangli0505/TEFTC-ICV/blob/main/tits_code.m)

The code contains the implementation used to reproduce the main simulation results reported in the paper.

---

## Citation

If you find this work useful for your research, please cite our paper:

```bibtex
@ARTICLE{11048674,
  author={Dong, Haoyang and Li, Yang and Wang, Lu and Wang, Xudong and Qin, Hongmao and Wan, Haiying and Bian, Yougang and Li, Yongfu},
  journal={IEEE Transactions on Intelligent Transportation Systems}, 
  title={Transmission-Efficient Fault-Tolerant Control for Intelligent and Connected Vehicles With Input Quantization and Event-Triggered Mechanism}, 
  year={2025},
  volume={26},
  number={10},
  pages={14953-14967},
  doi={10.1109/TITS.2025.3579038}
}
```

---

## Paper

**H. Dong, Y. Li, L. Wang, X. Wang, H. Qin, H. Wan, Y. Bian, and Y. Li**,
“Transmission-Efficient Fault-Tolerant Control for Intelligent and Connected Vehicles With Input Quantization and Event-Triggered Mechanism,”
*IEEE Transactions on Intelligent Transportation Systems*, vol. 26, no. 10, pp. 14953–14967, 2025.

**DOI:** [10.1109/TITS.2025.3579038](https://doi.org/10.1109/TITS.2025.3579038)

---

## Contact

For questions regarding the paper or the code, please contact lyxc56@gmail.com

Alternatively, please feel free to open an issue in this repository.

---

## License

This repository is provided for academic research purposes.

Please cite the corresponding paper if the code is used in your research.
