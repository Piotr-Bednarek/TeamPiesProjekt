% 1. Pobierz nazwę aktywnego modelu
system_name = gcs; 

% 2. Znajdź wszystkie bloki 'Goto' i 'From' w modelu
% 'LookUnderMasks' i 'FollowLinks' pozwalają szukać też głębiej w podsystemach
gotos = find_system(system_name, 'LookUnderMasks', 'on', 'FollowLinks', 'on', 'BlockType', 'Goto');
froms = find_system(system_name, 'LookUnderMasks', 'on', 'FollowLinks', 'on', 'BlockType', 'From');

% Połącz listy, aby przeanalizować wszystkie tagi
all_blocks = [gotos; froms];

% 3. Zdefiniuj dostępne kolory w Simulinku
% Simulink przyjmuje nazwy kolorów, a nie RGB wprost dla tła bloku
colors = {'cyan', 'magenta', 'yellow', 'lightBlue', 'orange', 'green', 'red', 'white', 'gray'};

% 4. Pobierz tagi ze wszystkich bloków
if ~isempty(all_blocks)
    tags = get_param(all_blocks, 'GotoTag');
    unique_tags = unique(tags); % Lista unikalnych nazw tagów

    % 5. Pętla po unikalnych tagach
    for i = 1:length(unique_tags)
        current_tag = unique_tags{i};
        
        % Wybierz kolor (jeśli tagów jest więcej niż kolorów, zapętlamy kolory)
        color_idx = mod(i-1, length(colors)) + 1;
        selected_color = colors{color_idx};
        
        % Znajdź bloki z tym konkretnym tagiem
        blocks_with_tag = find_system(system_name, 'LookUnderMasks', 'on', 'FollowLinks', 'on', 'GotoTag', current_tag);
        
        % Ustaw kolor tła
        for j = 1:length(blocks_with_tag)
            set_param(blocks_with_tag{j}, 'BackgroundColor', selected_color);
        end
    end
    
    disp(['Pokolorowano pomyślnie ' num2str(length(unique_tags)) ' unikalnych tagów.']);
else
    disp('Nie znaleziono bloków Goto/From.');
end