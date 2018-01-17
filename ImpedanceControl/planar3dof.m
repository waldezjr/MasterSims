function planar3dof = planar3dof()

% Simple 3DOF RRR manipulator
L(1) = Link([ 0 0 0.5	0	0], 'standard');
L(2) = Link([ 0 0 0.5	0	0], 'standard');
L(3) = Link([ 0 0 0.5	0	0], 'standard');

planar3dof = SerialLink(L, 'name', 'Planar RRR');

end