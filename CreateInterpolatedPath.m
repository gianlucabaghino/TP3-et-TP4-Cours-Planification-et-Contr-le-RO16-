function interpolated_path = CreateInterpolatedPath(path, r)
    % GÉNÉRATION D'UN CHEMIN INTERPOLÉ
    % Cette fonction génère un ensemble de points intermédiaires entre les points
    % d'un chemin donné. Elle permet de rendre le chemin plus dense pour guider le robot de manière plus précise.
    %
    % Entrée :
    %   path -> Ensemble de points définissant le chemin : [x1 x2 ... ; y1 y2 ...]
    %   r -> Distance cible entre deux points successifs sur le chemin interpolé
    %
    % Sortie :
    %   interpolated_path -> Chemin densifié avec des points interpolés entre chaque segment

    interpolated_path = [];
    n_points = size(path, 2);  % Nombre de points dans le chemin initial

    % Parcours de chaque segment du chemin
    for i = 1:n_points - 1
        % Points de départ et d'arrivée du segment
        p1 = path(:, i);
        p2 = path(:, i + 1);

        % Calcul de la longueur du segment
        segment_length = norm(p2 - p1);

        % Calcul du nombre de points intermédiaires nécessaires
        num_points = ceil(segment_length / r);

        % Génération des points interpolés entre p1 et p2
        x_interpolated = linspace(p1(1), p2(1), num_points);
        y_interpolated = linspace(p1(2), p2(2), num_points);

        % Ajout des points interpolés au chemin total
        interpolated_path = [interpolated_path, [x_interpolated; y_interpolated]];
    end
end

