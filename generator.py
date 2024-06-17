"""
###########################
######## Generator ########
###########################

This Python script generates random configurations for the 8-puzzle problem, converts these configurations into
the Planning Domain Definition Language (PDDL) format, and writes the problem definition to a file. The script
includes functions to validate positions, make moves within the puzzle, generate a randomized puzzle, and convert
the puzzle state to PDDL statements.
"""

import random

# Definition of valid positions and movements for the 8-puzzle
positions = [(i, j) for i in range(1, 4) for j in range(1, 4)]
tiles = [f't{i}' for i in range(1, 9)] + ['blank']

# Possible moves (up, down, left, right)
moves = {
    'up': (-1, 0),
    'down': (1, 0),
    'left': (0, -1),
    'right': (0, 1)
}

def is_valid_position(pos):
    """
    Check if a position is within the valid range for the 8-puzzle.
        :param pos: A tuple representing a position (row, column).
    :return: True if the position is valid, False otherwise.
    """
    return 1 <= pos[0] <= 3 and 1 <= pos[1] <= 3

def make_move(puzzle, blank_pos, move):
    """
    Move the blank tile in the puzzle if the move is valid.
        :param puzzle: A dictionary representing the current state of the puzzle.
        :param blank_pos: A tuple representing the current position of the blank tile.
        :param move: A tuple representing the move direction (row_change, column_change).
    :return: The new position of the blank tile after the move.
    """
    new_pos = (blank_pos[0] + move[0], blank_pos[1] + move[1])
    if is_valid_position(new_pos):
        puzzle[blank_pos], puzzle[new_pos] = puzzle[new_pos], puzzle[blank_pos]
        blank_pos = new_pos
    return blank_pos

def generate_puzzle(num_moves=100):
    """
    Generate a random puzzle configuration by making a series of valid moves.
        :param num_moves: The number of random moves to perform starting from the solved state.
    :return: A dictionary representing the puzzle state after the moves.
    """
    # Solved initial state
    puzzle = {
        (1, 1): 't1', (2, 1): 't2', (3, 1): 't3',
        (1, 2): 't4', (2, 2): 't5', (3, 2): 't6',
        (1, 3): 't7', (2, 3): 't8', (3, 3): 'blank'
    }
    blank_pos = (3, 3)

    for _ in range(num_moves):
        move = random.choice(list(moves.values()))
        blank_pos = make_move(puzzle, blank_pos, move)

    return puzzle

def puzzle_to_pddl(puzzle):
    """
    Convert a puzzle state to a list of PDDL (Planning Domain Definition Language) facts.
        :param puzzle: A dictionary representing the current state of the puzzle.
    :return: A list of strings representing the PDDL facts for the puzzle state.
    """
    pddl = []
    for pos, tile in puzzle.items():
        if tile != 'blank':
            pddl.append(f'(at {tile} p{pos[0]} p{pos[1]})')
        else:
            pddl.append(f'(blank p{pos[0]} p{pos[1]})')
    return pddl

def generate_random_problem(file_name, problem_id, solution_cost, num_moves=100):
    """
    Generate a random 8-puzzle problem in PDDL format and save it to a file.
        :param file_name: The name of the file to save the problem.
        :param problem_id: An identifier for the problem.
        :param solution_cost: The cost of the optimal solution for the problem.
        :param num_moves: The number of random moves to generate the initial puzzle state.
    """
    puzzle = generate_puzzle(num_moves)
    initial_state_pddl = puzzle_to_pddl(puzzle)
    
    goal_state = [
        '(at t1 p1 p1)',
        '(at t2 p2 p1)',
        '(at t3 p3 p1)',
        '(at t4 p1 p2)',
        '(at t5 p2 p2)',
        '(at t6 p3 p2)',
        '(at t7 p1 p3)',
        '(at t8 p2 p3)',
        '(blank p3 p3)',
        '(handEmpty rob)'
    ]

    problem_template = """
;; Eight puzzle problem #{problem_id}.
;; Optimal solution cost = {solution_cost}.

(define (problem korf{problem_id})
  (:domain strips-sliding-tile)
  (:objects t1 t2 t3 t4 t5 t6 t7 t8 
            p1 p2 p3
            rob)
  (:init
   (tile t1) (tile t2) (tile t3)
   (tile t4) (tile t5) (tile t6)
   (tile t7) (tile t8) 
   (position p1) (position p2) (position p3)
   (robot rob)
   (inc p1 p2) (inc p2 p3)
   (dec p3 p2) (dec p2 p1)
   (handEmpty rob)
   (at rob p3 p3)

   ;; initial state
   {initial_state}
  )

  ;; standard goal state
  (:goal
   (and
    {goal_state}
   ))
 )
""".format(problem_id=problem_id, solution_cost=solution_cost, 
           initial_state="\n   ".join(initial_state_pddl), 
           goal_state="\n    ".join(goal_state))
    
    with open(file_name, 'w') as file:
        file.write(problem_template)

# Generate a random problem file
generate_random_problem('Code/problem.pddl', 1, 57)
