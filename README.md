# Optimal RPOD Trajectories using Convex Optimization

Some tools to generate optimal trajectories using the convex optimization package available at http://cvxr.com/.

If any publication results from the use of this software, please cite reference [1], [2] as relevant, as well as the authors of CVX [3], [4]

Two core functions: 

1. SuccessiveContaminationReductionUsingCVX.m - Recreates results in conference paper[1] where we implement a thruster pointing constrained control problem and solve it using a method of successive approximations. The constraint is illustrated in the figure below.

<p align="center" width="100%">
    <img width="50%" src="diagramConstraint.png"> 
</p>

2. SOCP_OrtolanoResults.m - Replicates some results from Ortolano[2], namely rendezvous and proximity operations with a conical approach corridor and spherical keep-out zone using second order cone programming techniques.

**References**

[1] Panag H., Woollands R., Bandyopadhyay S. and Rahmani A., “Reducing Thruster Plume Contamination for Satellite Servicing Using Convex Optimization,” American Astronautical Society Guidance, Navigation and Controls Conference, Harvey Mamich, 2023, Paper AAS 23-016.

[2] Nicholas G Ortolano. Phd Thesis Autonomous Trajectory Planning for Satellite RPO and Safety of Flight Using Convex Optimization. PhD thesis, 2018.

[3] Michael Grant and Stephen Boyd. CVX: Matlab software for disciplined convex programming, version 2.0 beta. https://cvxr.com/cvx, September 2013.

[4] Michael Grant and Stephen Boyd. Graph implementations for nonsmooth convex programs, Recent Advances in Learning and Control (a tribute to M. Vidyasagar), V. Blondel, S. Boyd, and H. Kimura, editors, pages 95-110, Lecture Notes in Control and Information Sciences, Springer, 2008. http://stanford.edu/~boyd/graph_dcp.html.
