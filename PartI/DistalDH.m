function T = DistalDH(Link)

a = Link(1);
q1 = Link(2);
d = Link(3);
q2 = Link(4);

if isa(q1, 'sym')
    s1 = sin(q1);
    c1 = cos(q1);
else 
    s1 = sin(q1);
    c1 = cos(q1);
end

if isa(q2, 'sym')
    s2 = sin(q2);
    c2 = cos(q2);
else 
    s2 = sin(q2);
    c2 = cos(q2);
end

T = [ c2,-c1*s2,  s1*s2, a*c2;
      s2, c1*c2, -s1*c2, a*s2;
       0,    s1,     c1,    d;
       0,     0,      0,    1 ];

end


