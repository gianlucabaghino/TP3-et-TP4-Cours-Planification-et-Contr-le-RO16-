function ok = verif_stability(x_verif)
  try
    pkg load control
  end
  %parametres
  mu=0.5;
  u0=0;
  x10=0;
  x20=0;

  %matrice de poids
  Q=[0.5,0;0,0.5];
  R=[1];

  %TODO écrire les matrics A et B de la linéarisation Jacobienne
  %linéarisés jacobienne
  A = [u0 * (1 - mu), 1;
       1, -4 * u0 * (1 - mu)];

  B = [mu + (1 - mu) * x10;
       mu - 4 * (1 - mu) * x20];

  %TODO avec riccati, trouver une commande stabilisante
  %essai riccati : A'P+PA-PB inv(R) B'P + Q =0
  [x, l, g] = care(A, B, Q, R);
  K = -g

  %TODO calculer l'équation du système avec rebouclage
  %systeme rebouclage
  Ak = A + B * K;
  eigs_Ak = eig(Ak)
  M = [-1, 0; 0, -1] - Ak;
  detM = det(M)

  %TODO calculer la borne lambda et la borne alpha à 95 % de lambda
  %calcul de la borne, on retrouve bien celle de l'article
  lambda = -max(eigs(Ak))
  % borne a 95 %
  alpha = lambda-0.05*lambda

  %TODO écrire les matrices de l'équation de Lyapunov et la résoudre pour obtenir la matrice P
  %matrice pour equation lyap
  Al = (Ak+[alpha, 0;0, alpha])'
  Bl = (Q + K' * R * K)
  P = lyap(Al,Bl)

  %verif lyap
  Al*P + P*Al' + Bl;
  eigs_P = eigs(P)

  %ici on calcul la borne du problème quadratique beta
  [x1,obj] = qp([0.5;0.5],-2*P,[],[],[],[-0.8;-0.8],[0.8;0.8],-2,K,2)
  beta = -obj

  %TODO écrire le test qui valide ou non si le point est dans la zone de stabilité du controleur
  test = x_verif' * P * x_verif;
  ok = (test < beta);

endfunction
