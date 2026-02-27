import json

with open("geometry.json", "r") as g:
    GEOM = json.load(g)


def players_pos_relative_to_y(rod_id: int, relative_pos: float) -> list[float]:
    """
    Convers the relative position that the camera gives to the y of the euclidian pos.

    # Args:
        - rod_id - This is the id specified in the geometry.json file.
        - relative_pos - This is the vlue that the camera returns from 0 to 1.

    # Returns:
        - players_pos_y: The y values of all the players positions of a specific rod.

    # Raises:
        ValueError: If rod_id is invalid or relative_pos is outside [0, 1].
    """
    if not (1 <= rod_id <= 8):
        raise ValueError("rod_id should be between 1 and 8")

    if not (0 <= relative_pos <= 1):
        raise ValueError("relative_pos should be between 0 and 1")

    rods_geom: dict[str, int | str] = GEOM["rods"][rod_id - 1]

    players_pos_y: list[float] = []
    number_of_players: int = int(rods_geom["players"])
    first_offset: int = int(rods_geom["first_offset"])
    spacing: int = int(rods_geom["spacing"])
    travel: int = int(rods_geom["travel"])

    for i in range(0, number_of_players):
        player_min_y: int = first_offset + spacing * i
        player_max_y: int = player_min_y + travel
        player_y: float = player_min_y + (player_max_y - player_min_y) * relative_pos
        players_pos_y.append(player_y)

    return players_pos_y


def can_player_reach_y(rod_id: int, player_id: int, y: float) -> bool:
    """
    Check if a given player can reach the y position.

    # Args:
        - rod_id - This is the id specified in the geometry.json file.
        - player_id: the id of the player. Top player has id 1, bottom player can have at most 5.
        - y: the value that we want the player to be at

    # Returns:
        - relative_rod_position: The position that the rod has to be in, so that that specific player has the right y value. If player can not reach that y value, raises ValueError.

    # Raises:
        ValueError if rod id is not between 1 and 8, if player id is too high or low and if y is unreachable.

    """
    if not (1 <= rod_id <= 8):
        raise ValueError("rod_id should be between 1 and 8")

    rods_geom: dict[str, int | str] = GEOM["rods"][rod_id - 1]
    number_of_players: int = int(rods_geom["players"])

    if not (1 <= player_id <= number_of_players):
        raise ValueError(f"player_id should be between 1 and {number_of_players}")

    first_offset: int = int(rods_geom["first_offset"])
    spacing: int = int(rods_geom["spacing"])
    travel: int = int(rods_geom["travel"])

    player_min_y: int = first_offset + spacing * (player_id - 1)
    player_max_y: int = player_min_y + travel

    return (player_min_y <= y <= player_max_y)


def player_pos_y_to_relative(rod_id: int, player_id: int, y: float) -> float:
    """
    Convers the euclidian y to the relavie position that can be sent to the server

    # Args:
        - rod_id - This is the id specified in the geometry.json file.
        - player_id: the id of the player. Top player has id 1, bottom player can have at most 5.
        - y: the value that we want the player to be at

    # Returns:
        - relative_rod_position: The position that the rod has to be in, so that that specific player has the right y value. If player can not reach that y value, raises ValueError.

    # Raises:
        ValueError if rod id is not between 1 and 8, if player id is too high or low and if y is unreachable.
    """

    if not (1 <= rod_id <= 8):
        raise ValueError("rod_id should be between 1 and 8")

    rods_geom: dict[str, int | str] = GEOM["rods"][rod_id - 1]
    number_of_players: int = int(rods_geom["players"])

    if not (1 <= player_id <= number_of_players):
        raise ValueError(f"player_id should be between 1 and {number_of_players}")

    first_offset: int = int(rods_geom["first_offset"])
    spacing: int = int(rods_geom["spacing"])
    travel: int = int(rods_geom["travel"])

    player_min_y: int = first_offset + spacing * (player_id - 1)
    player_max_y: int = player_min_y + travel

    if not (player_min_y <= y <= player_max_y):
        raise ValueError(f"y should be between {player_min_y} and {player_max_y} for this specific player. y: {y}, rod_id: {rod_id}, player_id: {player_id}")

    relative_pos: float = (y - player_min_y) / (player_max_y - player_min_y)

    return relative_pos


if __name__ == "__main__":
    print(players_pos_relative_to_y(1, 0))
    print(players_pos_relative_to_y(1, 1))

    print(can_player_reach_y(1, 1, 257))
    print(can_player_reach_y(1, 1, 258))
