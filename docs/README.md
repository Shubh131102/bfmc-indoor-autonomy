# Documentation

Project documentation, reports, and presentation materials for BFMC autonomous vehicle.

## Contents

**Project_Report.pdf**
- Complete internship project report
- Traffic sign detection implementation
- FSM controller design
- Experimental results and analysis

**Presentation.pdf** (or .pptx)
- Project overview slides
- System architecture
- Key results and demonstrations

**Architecture_Diagram.png**
- System component overview
- Data flow between modules
- ROS2 node communication

## Project Overview

**Bosch Future Mobility Challenge (BFMC)**
- Indoor autonomous navigation with traffic rule compliance
- Raspberry Pi-based control system
- ROS2 Humble framework
- Real-time traffic sign detection and response

## Key Achievements

- 85% traffic rule compliance rate
- 20 Hz traffic sign detection
- 92% detection accuracy
- Real-time FSM control at 50 Hz
- Successful sim-to-real transfer (88% success rate)

## System Components

1. **Traffic Sign Detection:** CNN-based real-time detection (12 classes)
2. **Finite State Machine:** State-based control (CRUISE, STOP, YIELD, etc.)
3. **Sensor Fusion:** Camera + LiDAR + IMU integration
4. **Path Planning:** A* global + DWA local planning
5. **Safety Layer:** Emergency braking and obstacle avoidance

## Related Links

- BFMC Official: [Bosch Future Mobility Challenge](https://www.boschfuturemobility.com/)
- Repository: [GitHub](https://github.com/Shubh131102/bfmc-project)

## Citation
```bibtex
@techreport{jangle2024bfmc,
  title={Autonomous Vehicle Navigation with Traffic Rule Compliance},
  author={Jangle, Shubham},
  institution={Ahmedabad University},
  year={2024}
}
```
