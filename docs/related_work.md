# Related work — 20 peer-reviewed papers (2024–2025)

Compiled 2026-09-20 for *"Towards Autonomous Industrial Mobility: An AI Framework for Optimal
Path Planning and Resource-Efficient Navigation"*.

**Selection rules applied:**
- Journal articles and conference/proceedings papers only. **No arXiv preprints.**
- **No 2026 publications.** 2025 preferred, 2024 accepted where no 2025 equivalent exists.
- Every DOI, title, author list and year below was read from Crossref publisher metadata on the
  compile date, not written from memory. The AAMAS entry was additionally checked by resolving
  its DOI to the ACM Digital Library.

Final split: **15 from 2025, 5 from 2024.**

---

## A. Neuro-symbolic and interpretable decision policies (WP5/WP6/WP9)

### 1. Neural DNF-MT: A Neuro-symbolic Approach for Learning Interpretable and Editable Policies
Kexin Gu Baugh, Luke Dickens, Alessandra Russo — **AAMAS 2025** (Int. Joint Conf. on Autonomous
Agents and Multiagent Systems), proceedings article. DOI 10.65109/spbp4627
https://doi.org/10.65109/spbp4627 · ACM DL: https://dl.acm.org/doi/10.5555/3709347.3743538
*Why it matters here:* the closest peer-reviewed analogue to our architecture — a learned policy
whose symbolic part stays readable and **editable** after training. Our symbolic layer is
hand-written rather than learned, which is a difference worth defending explicitly in the
discussion. Top-tier venue; use this as the anchor citation for the neuro-symbolic claim.

### 2. Supporting Human-Robot Collaboration and Safety with the Proposed Explainable Neuro-Symbolic Reasoning
Rafał Kozik, Aleksandra Pawlicka, Marek Pawlicki et al. — **LNCS, Availability, Reliability and
Security (ARES) 2025**, book chapter. DOI 10.1007/978-3-032-00642-4_15
https://doi.org/10.1007/978-3-032-00642-4_15
*Why:* argues that safety in industrial robotics must be symbolically auditable, not statistically
inferred. Direct support for our WP9 result that only the rule-carrying policies were safe in all
1440 shifts.

### 3. Adaptive and transparent decision-making in autonomous robots through graph-structured world models
Site Hu, Takato Horii, Takayuki Nagai — **Advanced Robotics**, 2024. DOI 10.1080/01691864.2024.2415995
https://doi.org/10.1080/01691864.2024.2415995
*Why:* transparency obtained from an explicit structured model rather than post-hoc explanation —
the same stance as our per-decision explanation strings ("[routine] A has 2 units waiting; neural
score −0.89, chosen over PICKUP:B by +0.34").

---

## B. Safe learning, shielding and action masking (the PPO comparison and the ±1 bound)

### 4. Shields for Safe Reinforcement Learning
Bettina Könighofer, Roderick Bloem, Nils Jansen, Sebastian Junges, Stefan Pranger —
**Communications of the ACM**, 68(11), pp. 80–90, 2025. DOI 10.1145/3715958
https://doi.org/10.1145/3715958
*Why:* the authoritative, citable statement of shielding — a correctness layer that filters a
learned policy's actions. Our hard rules are exactly a domain-specific shield. Use this for the
formal vocabulary and to justify why safety should not be left to the reward function.

### 5. Towards robust shielded reinforcement learning through adaptive constraints and exploration: the fear field framework
Haritz Odriozola-Olalde, Maider Zamalloa, Nestor Arana-Arexolaleiba, Jon Perez-Cerrolaza —
**Engineering Applications of Artificial Intelligence**, 2025. DOI 10.1016/j.engappai.2025.110055
https://doi.org/10.1016/j.engappai.2025.110055
*Why:* shielding with *adaptive* constraint strength — conceptually the nearest published relative
of our bounded neural correction, where the network keeps influence but cannot exceed ±1 score
unit. Cite it next to the bound-selection table.

### 6. Dual Resource Scheduling Method of Production Equipment and Rail-Guided Vehicles Based on Proximal Policy Optimization Algorithm
Nengqi Zhang, Bo Liu, Jian Zhang et al. — **Technologies**, 13(12):573, 2025.
DOI 10.3390/technologies13120573
https://doi.org/10.3390/technologies13120573
*Why:* action-mask-constrained PPO deciding jointly about production equipment and a transport
vehicle — the published peer-reviewed counterpart of our MaskablePPO baseline, in our exact
problem shape (production + transport in one decision). Essential for the PPO comparison chapter.

### 7. Human-Risk-Aware Safe Path Planning Based on Reinforcement Learning for Autonomous Mobile Robots
Zhongjie Long, Xianbo Zhang, Jian Mi et al. — **Sensors**, 25(23):7211, 2025. DOI 10.3390/s25237211
https://doi.org/10.3390/s25237211
*Why:* RL path planning with an explicit safety criterion on an AMR. A comparison point for how
others encode safety when they do not have a symbolic layer.

---

## C. Energy- and battery-aware AMR / AGV decision making (WP2/WP4/WP5)

### 8. Scheduling mobile robots in dynamic production environments considering battery management constraints
Maximilian Dilefeld, Thorsten Claus, Frank Herrmann, Thorsten Schmidt, Enrico Teich —
**Logistics Research**, 18(1–2), 2025. DOI 10.1108/lore-12-2024-0017
https://doi.org/10.1108/lore-12-2024-0017
*Why:* the single closest paper to our decision problem — online scheduling with a utility function
that weighs "do the transport task" against "go and charge", under uncertainty about future tasks.
That is precisely the trade-off our symbolic priority tiers plus H5 rule encode. Their solution is
an auction; ours is rules plus a bounded learned ranking. Best head-to-head comparison in the list.

### 9. Deep reinforcement learning for solving efficient and energy-saving flexible job shop scheduling problem with multi-AGV
Weiyao Cheng, Chaoyong Zhang, Leilei Meng, Biao Zhang, Kaizhou Gao, Hongyan Sang —
**Computers & Operations Research**, 2025. DOI 10.1016/j.cor.2025.107087
https://doi.org/10.1016/j.cor.2025.107087
*Why:* throughput and energy in one objective, solved by DRL, with AGV transport in the loop.
Our objective (+1 delivered, −1 lost, −0.5/Wh, −20/violation) is the single-robot version of this.

### 10. Energy-efficient and self-adaptive AGV scheduling approach based on hierarchical reinforcement learning for flexible shop floor
Xiao Chang, Xiaoliang Jia, Hao Hu — **Computers & Industrial Engineering**, 2025.
DOI 10.1016/j.cie.2025.111140
https://doi.org/10.1016/j.cie.2025.111140
*Why:* explicitly benchmarks learned scheduling against dispatching rules (STD, FCFS, LWT). That is
our `rule` baseline in an industrial-engineering framing, and it supports our finding that the rule
policy is the most energy-efficient but collapses when time is short.

### 11. A Reinforcement Learning-Based AGV Scheduling for Automated Container Terminals With Resilient Charging Strategies
Shaorui Zhou, Yeyi Yu, Min Zhao et al. — **IET Intelligent Transport Systems**, 2025.
DOI 10.1049/itr2.70027
https://doi.org/10.1049/itr2.70027
*Why:* charging treated as a first-class, learned decision rather than a fixed threshold — the
published justification for our WP5 change from one fixed CHARGE action to CHARGE 40/70/100 %.

### 12. Digital twin-driven deep reinforcement learning for real-time optimisation in dynamic AGV systems
Donggun Lee, Yong-Shin Kang, Sang Do Noh — **International Journal of Production Research**, 2025.
DOI 10.1080/00207543.2025.2543491
https://doi.org/10.1080/00207543.2025.2543491
*Why:* the same two-layer method as ours — train and decide against a fast digital model, then run
the result on the real system. Cite alongside our fast-simulator-vs-Gazebo validation (0.6 % on
distance/time/energy).

### 13. Online Planning for Autonomous Mobile Robots with Different Objectives in Warehouse Commissioning Task
Satoshi Warita, Katsuhide Fujita — **Information**, 15(3):130, 2024. DOI 10.3390/info15030130
https://doi.org/10.3390/info15030130
*Why:* online (not offline-optimal) AMR planning under competing objectives in a warehouse. Matches
our setting: the robot decides at each step with no knowledge of future production.

---

## D. Energy models and their calibration — justifying `params.yaml` (WP2 revision)

### 14. Energy Utilization Prediction Techniques for Heterogeneous Mobile Robots: A Review
Krystian Góra, Grzegorz Granosik, Bartłomiej Cybulski — **Energies**, 17(13):3256, 2024.
DOI 10.3390/en17133256
https://doi.org/10.3390/en17133256
*Why:* places our analytic drive + idle + payload model among the published alternatives and gives
Wh/km ranges to defend our 9.3 Wh/km figure against. Use it where the WP2-revision table corrects
the old 0.35 Wh/m value.

### 15. Cross-Validated Neural Network Optimization for Explainable Energy Prediction in Industrial Mobile Robots
Danel Rico-Melgosa, Ekaitz Zulueta, Jorge Rodriguez-Guerra et al. — **Applied Sciences**,
15(23):12644, 2025. DOI 10.3390/app152312644
https://doi.org/10.3390/app152312644
*Why:* finds angular velocity and linear acceleration to be the dominant energy predictors, with
payload only secondary. This is the measured justification for WP8's turning-rate metric, and it
independently explains why our planner comparison found only a ~2 % energy difference between
planners that drive almost the same distance.

### 16. A Hybrid Parameters Estimation Approach for Power Consumption Modeling of Ground Mobile Robots With Unknown Payload
Parham Haji Ali Mohamadi, Amin Khorasani, Tom Verstraten et al. — **Journal of Field Robotics**,
2024. DOI 10.1002/rob.22470
https://doi.org/10.1002/rob.22470
*Why:* payload-dependent power for ground robots. The published counterpart of our
`payload_wh_per_m_per_kg = drive energy / robot mass`, so cite it to justify that coefficient
rather than presenting it as an assumption.

---

## E. ROS 2 / Nav2 and simulation-to-reality validation (WP1/WP3/WP8/WP9)

### 17. A ROS 2-Based Navigation and Simulation Stack for the Robotino
Saurabh Borse, Tarik Viehmann, Alexander Ferrein, Gerhard Lakemeyer — **LNCS, RoboCup 2024: Robot
World Cup XXVII**, 2025. DOI 10.1007/978-3-031-85859-8_2
https://doi.org/10.1007/978-3-031-85859-8_2
*Why:* ROS 2 + Nav2 in a logistics environment, with simulation and real-robot runs reported side
by side. The published reference for our stack choice, and for reporting sim/real consistency.

### 18. Evaluating mobile robot navigation behavior in flexible assembly systems through digital twin and real-world experiments
Lukas Bergs, Meike Huber, Alexander Moriz et al. — **Discover Robotics**, 2025.
DOI 10.1007/s44430-025-00010-4
https://doi.org/10.1007/s44430-025-00010-4
*Why:* a multi-metric methodology (localisation accuracy, path consistency, goal accuracy,
navigation performance) for quantifying the simulation-to-reality gap. Methodologically the closest
published match to our WP9 tier-2 table, where the gap was −5.7 % on score and +0.2 % on energy.

---

## F. Surveys for the related-work chapter

### 19. Path Planning Trends for Autonomous Mobile Robot Navigation: A Review
Yuexia Tang, Muhammad Aizzat Zakaria, Maryam Younas — **Sensors**, 25(4):1206, 2025.
DOI 10.3390/s25041206
https://doi.org/10.3390/s25041206
*Why:* the current survey to open the related-work chapter with and to position Theta*/NavFn/Smac
within the wider field.

### 20. Advancing mobile robot navigation with DRL and heuristic rewards: A comprehensive review
Mazbahur Rahman Khan, Azhar Mohd Ibrahim, Suaib Al Mahmud, Farah Asyiqin Samat, Farahiyah Jasni,
Muhammad Imran Mardzuki — **Neurocomputing**, 2025. DOI 10.1016/j.neucom.2025.131036
https://doi.org/10.1016/j.neucom.2025.131036
*Why:* surveys exactly the learned-vs-heuristic boundary this thesis sits on. Use it to argue that
the field's usual hybrid is "heuristics shaping the reward", whereas ours is "rules constraining
the action set" — a different and stronger guarantee.

---

## Runners-up (verified, peer-reviewed, 2024–2025)

| Paper | Link | Use for |
|---|---|---|
| Katona, Neamah, Korondi — *Obstacle Avoidance and Path Planning Methods for Autonomous Navigation of Mobile Robot*, **Sensors** 24(11):3573, 2024 | https://doi.org/10.3390/s24113573 | Background on local/global planning if ch. 2 needs more depth |
| Aremu, Kabir, Ahmed — *Autonomous Mobile Robot Path Planning Techniques — A Review: Classical and Heuristic Techniques*, **IEEE Access**, 2025 | https://doi.org/10.1109/access.2025.3579863 | Alternative survey; classical-planner taxonomy |
| Khan, Chaudhari, Ramesh et al. — *Neuro-Symbolic Reinforcement Learning for Context-Aware Decision Making in Safe Autonomous Vehicles*, **IJACSA** 16(5), 2025 | https://doi.org/10.14569/IJACSA.2025.0160558 | Architecturally very close to ours (symbolic rules + neural ranking + explanation), **but IJACSA is a weak, low-selectivity venue.** Cite only as supporting evidence, never as the anchor for the neuro-symbolic claim. |

---

## Excluded by your rules — recorded so the decision is traceable

These are all closely related and would otherwise have made the list. Each was dropped for a stated
reason, so they can be reinstated if the constraints change before submission.

| Paper | Venue | Why excluded |
|---|---|---|
| Macenski, Booker, Wallace, Fischer — *Cost-Aware Kinematically Feasible Planning for Mobile and Surface Robotics* | IEEE Robotics and Automation Practice | Peer-reviewed, but **2026** |
| Bischoff, Rinciog, Meyer — *Reinforcement Learning for AMR Charging Decisions: The Impact of Reward and Action Space Design* | LNCS (LION) | Peer-reviewed, but **2026** |
| Mao, Xie, Fang — *In-plant autonomous mobile robot scheduling and routing problem considering battery consumption model* | Transportation Research Part C | Peer-reviewed, but **2026** |
| Luo, Lu, He — *Energy-Constrained Hybrid Repair for Lifelong Multi-Agent Path Finding in Smart Warehouses* | Electronics | Peer-reviewed, but **2026** |
| Zhang, Yan, Hu — *Deep reinforcement learning for dynamic scheduling of energy-efficient automated guided vehicles* | Journal of Intelligent Manufacturing | Peer-reviewed, but **2023** |
| Alt, Dvorak, Katic et al. — *BANSAI: Bridging the AI Adoption Gap in Industrial Robotics with Neurosymbolic Programming* | — | **arXiv preprint only** |
| Stappert, Lutz, Goby, Neumann — *Integrating Human Knowledge Through Action Masking in RL for Operations Research* | — | **arXiv preprint only** |
| Seo, Nakamura, Bajcsy — *Uncertainty-aware Latent Safety Filters for Avoiding Out-of-Distribution Failures* | — | **arXiv preprint only** |
| Chavan, Joshi, Brocanelli — *Rethinking Energy Management for Autonomous Ground Robots on a Budget* | — | **arXiv preprint only** |
| Keith & La — *Review of Autonomous Mobile Robots for the Warehouse Environment* | — | **arXiv preprint only** |

Two of these hurt more than the others, and the gap should be acknowledged in the text rather than
hidden: **Macenski et al.** is the primary source for the Smac planners benchmarked in WP8 (its
Nav2 documentation can be cited instead), and **Chavan et al.** is the closest published support for
our finding that electronics idle power dominates a small robot's energy budget (6 of 7 Wh/h in
`balanced`) — for that claim, reference 14 (Energies 2024) is the peer-reviewed substitute.

---

## Gap statement this list supports

None of the 20 combines all four of: (a) a symbolic layer making safety **structural** rather than
learned, (b) a **bounded** neural correction on top of it, (c) a physically calibrated
energy/thermal/wear model, and (d) validation of the **same policy object** in a fast surrogate
simulator *and* in Gazebo with a full Nav2 stack.

The scheduling papers (8–13) optimise energy and throughput without interpretability or a safety
guarantee; the neuro-symbolic papers (1–3) give interpretability without an industrial resource
model; the safe-RL papers (4–7) give the safety machinery without an energy objective or a real
navigation stack; the energy papers (14–16) model consumption without deciding anything. That
intersection is the thesis's claim to novelty.
