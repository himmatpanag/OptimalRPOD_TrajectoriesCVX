# Optimal RPOD Trajectories using Convex Optimization

Some tools to generate optimal trajectories using the convex optimization package available at http://cvxr.com/

Two core functions: 

1. SuccessiveContaminationReductionUsingCVX.m - Recreates results in conference paper[1] where we implement a thruster pointing constrained control problem and solve it using a method of successive approximations. The constraint is illustrated in the figure below.

<p align="center" width="100%">
    <img width="50%" src="diagramConstraint.png"> 
</p>

2. SOCP_OrtolanoResults.m - Replicates some results from Ortolano[2], namely rendezvous and proximity operations with a conical approach corridor and spherical keep-out zone using second order cone programming techniques.


**References**
[1] H. Panag, R. Woollands, S. Bandyopadhyay, A. Rahmani; Reducing Thruster Plume Contamination for Satellite Servicing Using Convex Optimization, 45th AAS Guidance, Navigation & Controls Conference, 2023. 
[2] Nicholas G Ortolano. Phd Thesis Autonomous Trajectory Planning for Satellite RPO and Safety of Flight Using Convex Optimization. PhD thesis, 2018.

