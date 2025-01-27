function [control_output] = BicycleToPathControl(current_pose, path)
    % CONTRÔLEUR DE BICYCLETTE POUR SUIVRE UN CHEMIN
    % Cette fonction calcule les commandes de contrôle (vitesse linéaire et angle de direction)
    % pour qu'un robot de type bicyclette suive un chemin défini par un ensemble de points.
    %
    % Entrées :
    %   current_pose -> Pose actuelle du robot [x; y; theta] (position et orientation)
    %   path -> Ensemble de points définissant le chemin : [x1 x2 ... ; y1 y2 ...]
    %
    % Sortie :
    %   control_output -> Commandes de contrôle [v; phi], où :
    %       v = Vitesse linéaire
    %       phi = Angle de direction

    % Définir les gains du contrôleur
    Kp = 3;    % Gain proportionnel pour la distance
    Ka = 5;    % Gain proportionnel pour l'angle
    dp = 0.6;   % Distance entre les points du chemin
    dm = 0.3;   % Distance minimale avant de changer d'objectif

    % Initialisation persistante
    persistent interpolated_path goal_idx;

    if isempty(interpolated_path)
        % Générer le chemin interpolé au début de l'exécution
        interpolated_path = CreateInterpolatedPath(path, dp);
        goal_idx = 1;
    end

    % Objectif actuel : point cible sur le chemin interpolé
    goal_position = interpolated_path(:, goal_idx);

    % Calcul de la distance au point objectif
    distance_to_goal = norm(goal_position(1:2) - current_pose(1:2));

    % Calcul de l'angle vers le point objectif
    angle_to_goal = atan2(goal_position(2) - current_pose(2), goal_position(1) - current_pose(1));
    alpha = AngleWrap(angle_to_goal - current_pose(3));  % Erreur d'orientation par rapport à l'objectif

    % Calcul de la vitesse et de l'angle de direction avec contrôle proportionnel
    linear_velocity = Kp * distance_to_goal;   % Commande de vitesse
    steering_angle = Ka * alpha;   % Commande de direction

    % Appliquer des limites pour éviter des valeurs extrêmes
    linear_velocity = min(max(linear_velocity, -1), 1);   % Limiter la vitesse
    steering_angle = min(max(steering_angle, -1.2), 1.2); % Limiter l'angle de direction

    % Si l'on est assez proche du point actuel, passer au point suivant
    if distance_to_goal < dm && goal_idx < size(interpolated_path, 2)
        goal_idx = goal_idx + 1;
    end

    % Retourner les commandes de contrôle
    control_output = [linear_velocity; steering_angle];
end


