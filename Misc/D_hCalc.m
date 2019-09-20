%%

clear;

d = sym('d');
ct = sym('ct');
cp = sym('cp');
st = sym('st');
sp = sym('sp');

xd = d*ct*cp;
yd = d*st*cp;
zd = d*sp;

M = [sym('r11'), sym('r12'), sym('r13');
     sym('r21'), sym('r22'), sym('r23');
     sym('r31'), sym('r32'), sym('r33')];
t = [sym('px'); sym('py'); sym('pz')];
% T = [M, t; 0, 0, 0, 1];
 
X = M*[xd; yd; zd] + t;

x = X(1); y = X(2); z = X(3);

cx = sym('cx');
cy = sym('cy');
cz = sym('cz');
ax = sym('ax');
ay = sym('ay');
az = sym('az');

eqn = (((x - ax)^2)/(cx^2)) + (((y - ay)^2)/(cy^2)) + (((z - az)^2)/(cz^2)) == 1;

solve(eqn, d)