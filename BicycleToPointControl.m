function [control_output] = BicycleToPointControl(current_pose, target_point)
    % CONTRÔLE BICYCLETTE POUR ATTEINDRE UN POINT
    % Cette fonction calcule les commandes de contrôle (vitesse linéaire et angle de direction)
    % pour qu'un robot de type bicyclette atteigne un point cible spécifique.
    %
    % Entrées :
    %   current_pose -> Pose actuelle du robot [x; y; theta] (position et orientation)
    %   target_point -> Point cible désiré [x_goal; y_goal]
    %
    % Sortie :
    %   control_output -> Commandes de contrôle [v; phi], où :
    %       v = Vitesse linéaire
    %       phi = Angle de direction

    % Définir les gains du contrôleur
    K_rho = 20;    % Gain pour le contrôle de la distance
    K_alpha = 10;  % Gain pour l'alignement angulaire

    % Calculer la distance jusqu'au point cible (distance euclidienne)
    % rho = distance_to_target
    distance_to_target = sqrt((target_point(1) - current_pose(1))^2 + (target_point(2) - current_pose(2))^2);

    % Calculer l'angle vers le point cible par rapport à l'orientation du robot
    % alpha = angle_error
    angle_to_target = atan2(target_point(2) - current_pose(2), target_point(1) - current_pose(1));
    angle_error = AngleWrap(angle_to_target - current_pose(3));

    % Appliquer un contrôle proportionnel pour les vitesses linéaire et angulaire
    % v = vitesse linéaire, phi = angle de direction
    linear_velocity = K_rho * distance_to_target; % Commande de translation
    steering_angle = K_alpha * angle_error;       % Commande de direction

    % Appliquer des limites de saturation pour s'assurer que les valeurs de contrôle respectent les spécifications du modèle de bicyclette
    linear_velocity = min(max(linear_velocity, -1), 1);       % Limiter la vitesse linéaire
    steering_angle = min(max(steering_angle, -1.2), 1.2);     % Limiter l'angle du volant

    % Retourner les commandes de contrôle en sortie
    control_output = [linear_velocity; steering_angle];
end


