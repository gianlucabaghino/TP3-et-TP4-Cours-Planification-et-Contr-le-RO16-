function [ control_output ] = UnicycleToPoseControl( current_pose, target_pose )
    % CONTRÔLE UNICYCLE POUR ATTEINDRE UNE POSE
    % Cette fonction calcule les commandes de contrôle (vitesse linéaire et angulaire)
    % pour qu'un robot unicycle atteigne une pose cible spécifique.
    %
    % Entrées :
    %   current_pose -> Pose actuelle du robot [x; y; theta] (position et orientation)
    %   target_pose -> Pose cible désirée [x_goal; y_goal; theta_goal]
    %
    % Sortie :
    %   control_output -> Commandes de contrôle [v; w], où :
    %       v = Vitesse linéaire
    %       w = Vitesse angulaire

    % Définir les gains du contrôleur
    K_rho = 20;    % Gain pour le contrôle de la distance
    K_alpha = 10;  % Gain pour l'alignement angulaire
    K_beta = 5;    % Gain pour l'ajustement de l'orientation finale

    % Calculer la distance jusqu'à la cible (distance euclidienne)
    % rho = distance_to_goal
    distance_to_goal = sqrt((target_pose(1) - current_pose(1))^2 + (target_pose(2) - current_pose(2))^2);

    % Initialiser les variables de contrôle
    linear_velocity = 0;  % Vitesse linéaire (v)
    angular_velocity = 0; % Vitesse angulaire (w)

    % Vérifier si le robot est éloigné de la cible
    if (distance_to_goal > 0.05)
        % Calculer l'angle vers la cible par rapport à l'orientation du robot
        angle_to_target = atan2(target_pose(2) - current_pose(2), target_pose(1) - current_pose(1));
        % alpha = alignment_error
        alignment_error = AngleWrap(angle_to_target - current_pose(3));

        % Définir le seuil maximal pour l'alignement angulaire
        % alpha_max = max_alignment_error
        max_alignment_error = pi;

        % Appliquer un contrôle proportionnel pour les vitesses linéaire et angulaire
        linear_velocity = K_rho * distance_to_goal; % Avancer vers la cible
        angular_velocity = K_alpha * alignment_error; % S'aligner sur la direction cible

        % Arrêter le mouvement en translation si l'erreur d'alignement est trop grande
        if (abs(alignment_error) > max_alignment_error)
            linear_velocity = 0; % Pause en translation pour corriger l'orientation
        end

    else
        % Le robot est proche de la cible, ajuster son orientation finale
        % beta = orientation_error
        orientation_error = AngleWrap(target_pose(3) - current_pose(3));

        % Contrôle proportionnel pour ajuster la vitesse angulaire
        angular_velocity = K_beta * orientation_error;
    end

    % Appliquer des limites de saturation pour s'assurer que les valeurs de contrôle sont dans les bornes
    linear_velocity = min(max(linear_velocity, -1), 1); % Limiter la vitesse linéaire (v)
    angular_velocity = min(max(angular_velocity, -pi), pi); % Limiter la vitesse angulaire (w)

    % Retourner les commandes de contrôle en sortie
    control_output = [linear_velocity; angular_velocity];
end


