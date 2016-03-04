% ./alma run false keyboard true alfile demo/julia_location.pl debug 0 /tmp/debug

% juliaAt(1,1,3).
juliaAt(1,1,4).

if(and(juliaAt(X,Y,Z), juliaAt(A,B,C)), 
	and(and(eval_bound(X == A, [X,A]), eval_bound(Y == B, [Y,B])), eval_bound(Z == C, [Z,C]))).

% if there is a contradiction

fif(contra(X,Y,Z), conclusion(juliaMoved(A,B,C))).
