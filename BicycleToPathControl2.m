function [ u ] = BicycleToPathControl2( xTrue, Path )
% Calcule un contrôle pour suivre un chemin pour un modèle de vélo
%   xTrue est la position actuelle du robot : [ x y theta ]'
%   Path est l'ensemble des points définissant le chemin : [ x1 x2 ...
%                                                            y1 y2 ...]
%   u est le contrôle calculé : [v phi]'

persistent goalWaypointId xGoal; % Variables persistantes pour mémoriser l'état

% Initialisation du premier point de passage et de l'objectif au début
if xTrue == [0; 0; 0] % Si la position initiale est (0, 0, 0)
    goalWaypointId = 1; % On commence par le premier point du chemin
    xGoal = Path(:, 1); % On définit l'objectif comme le premier point
end

rho = 0.3; % Rayon de proximité pour considérer qu'un point est atteint
dt = 0.01; % Temps d'intégration (identique à celui utilisé dans la simulation)

%% Définir les objectifs et la taille de la fenêtre d'anticipation
window_size = 1000;
vmax = 2.0; % Vitesse maximale autorisée
dmax = vmax * dt; % Distance maximale parcourue en une étape

list_points = []; % Liste pour stocker les points prédits
xtemp = xTrue; % Position temporaire, initialisée à la position actuelle

%% Remplir la liste des points prédits en suivant le chemin
% On parcourt le chemin jusqu'à atteindre le nombre de points prédits spécifié par "window_size"
while size(list_points, 2) < window_size
    % Calcul de la distance entre la position actuelle et le point objectif
    if norm((Path(:, goalWaypointId) - xtemp)(1:2)) < rho
        % Si le point est suffisamment proche, on le considère atteint
        xtemp = Path(:, goalWaypointId); % Mise à jour de la position temporaire
        list_points = [list_points, xtemp]; % Ajout du point à la liste des points prédits
        goalWaypointId = goalWaypointId + 1; % Passage au point suivant du chemin
        goalWaypointId = min(goalWaypointId, size(Path, 2)); % Limite pour ne pas dépasser le chemin
    else
        % Si le point n'est pas encore atteint, calcul de la direction à suivre
        direction = Path(:, goalWaypointId) - xtemp; % Vecteur direction vers le point objectif
        direction = direction / norm(direction); % Normalisation pour obtenir une direction unitaire
        xtemp = xtemp + dmax * direction; % Déplacement de la position temporaire dans la direction
        list_points = [list_points, xtemp]; % Ajout du point prédictif à la liste
    endif
end

anticipation = window_size; % L'objectif est défini comme le dernier point de la fenêtre

%% Contrôle proportionnel (P-Control)
Krho = 10; % Gain proportionnel pour la distance à l'objectif
Kalpha = 5; % Gain proportionnel pour l'orientation vers l'objectif

% Calcul de l'erreur entre la position actuelle et le point anticipé
error = list_points(:, anticipation) - xTrue; % Erreur de position (x, y, theta)
goalDist = norm(error(1:2)); % Distance au point objectif (x et y uniquement)
AngleToGoal = AngleWrap(atan2(error(2), error(1)) - xTrue(3)); % Angle vers l'objectif en tenant compte de l'orientation

% Lois de contrôle proportionnel
u(1) = Krho * goalDist / (window_size * 10); % Calcul de la vitesse (borne proportionnelle à la distance)
u(2) = Kalpha * AngleToGoal; % Calcul de l'angle de direction

end


