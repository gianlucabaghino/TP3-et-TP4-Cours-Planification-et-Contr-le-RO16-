function [control_output] = BicycleToPoseControl(current_pose, goal_pose)
    % CONTRÔLE BICYCLETTE POUR ATTEINDRE UNE POSITION
    % Cette fonction calcule les commandes de contrôle (vitesse linéaire et angle de direction)
    % pour qu'un robot de type bicyclette atteigne une position cible spécifiée.
    %
    % Entrées :
    %   current_pose -> Pose actuelle du robot [x; y; theta] (position et orientation)
    %   goal_pose -> Position cible désirée [x_goal; y_goal; theta_goal]
    %
    % Sortie :
    %   control_output -> Commandes de contrôle [v; phi], où :
    %       v = Vitesse linéaire
    %       phi = Angle de direction

    % Définir les gains du contrôleur
    K_rho = 20;    % Gain pour le contrôle de la distance
    K_alpha = 10;  % Gain pour l'alignement angulaire
    K_beta = -5;   % Gain pour le contrôle de l'orientation finale

    % Calculer la distance jusqu'à la position cible (distance euclidienne)
    % rho = distance_to_goal
    distance_to_goal = sqrt((goal_pose(1) - current_pose(1))^2 + (goal_pose(2) - current_pose(2))^2);

    % Calculer l'angle vers la position cible par rapport à l'orientation du robot
    % alpha = angle_error
    angle_to_goal = atan2(goal_pose(2) - current_pose(2), goal_pose(1) - current_pose(1));
    alpha = AngleWrap(angle_to_goal - current_pose(3));  % Erreur d'orientation par rapport à la cible

    % Calculer l'orientation finale par rapport à la cible
    % beta = orientation_error
    beta = AngleWrap(goal_pose(3) - angle_to_goal);

    % Appliquer un contrôle proportionnel pour les vitesses linéaire et angulaire
    % v = vitesse linéaire, phi = angle de direction
    linear_velocity = K_rho * distance_to_goal; % Commande de translation
    steering_angle = K_alpha * alpha + K_beta * beta; % Commande de direction

    % Appliquer des limites de saturation pour s'assurer que les valeurs de contrôle respectent les spécifications du modèle de bicyclette
    linear_velocity = min(max(linear_velocity, -1), 1);  % Limiter la vitesse linéaire
    steering_angle = min(max(steering_angle, -1.2), 1.2); % Limiter l'angle du volant

    % Retourner les commandes de contrôle en sortie
    control_output = [linear_velocity; steering_angle];
end


