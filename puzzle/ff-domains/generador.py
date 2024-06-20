#########################################
###### 8-Puzzle Problem Generator ######
#########################################

# This Python script generates random 8-puzzle problems in PDDL format. The 8-puzzle is a sliding puzzle
# with a 3x3 grid, containing tiles numbered 1-8 and a blank space. The goal is to arrange the tiles in
# numerical order by sliding them into the blank space.

# Key Components:
# 1. Positions and Moves: Defines valid positions and possible moves (up, down, left, right).
# 2. Core Functions:
#    - is_valid_position: Checks if a position is within grid bounds.
#    - make_move: Executes a valid move, updating the blank tile's position.
#    - generate_puzzle: Creates a random puzzle state from the solved state.
#    - puzzle_to_pddl: Converts the puzzle state to PDDL format.
# 3. Problem Generation:
#    - generate_random_problem: Creates a PDDL file for an 8-puzzle problem with a random initial state and a standard goal state.

# Example Usage: Generates a random 8-puzzle problem and saves it to 'problem.pddl':
# generate_random_problem('problem.pddl', 1, 57)

import random

# definición de posiciones y piezas del objeto
positions = [(i, j) for i in range(1, 4) for j in range(1, 4)]
tiles = [f't{i}' for i in range(1, 9)] + ['blank']

# definición de movimientos
moves = {
    'up': (-1, 0),
    'down': (1, 0),
    'left': (0, -1),
    'right': (0, 1)
}

# verificar si la solución es válida
def is_valid_position(pos):
    return 1 <= pos[0] <= 3 and 1 <= pos[1] <= 3

# generar un movimiento
def make_move(puzzle, blank_pos, move):
    new_pos = (blank_pos[0] + move[0], blank_pos[1] + move[1])
    if is_valid_position(new_pos):
        puzzle[blank_pos], puzzle[new_pos] = puzzle[new_pos], puzzle[blank_pos]
        blank_pos = new_pos
    return blank_pos

# mezclar el puzzle
def generate_puzzle(num_moves=100):
    # estado inicial
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
    pddl = []
    for pos, tile in puzzle.items():
        if tile != 'blank':
            pddl.append(f'(at {tile} p{pos[0]} p{pos[1]})')
        else:
            pddl.append(f'(blank p{pos[0]} p{pos[1]})')
    return pddl

def generate_random_problem(file_name, problem_id, solution_cost, num_moves=100):
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

# generar un archivo de problema aleatorio
generate_random_problem('Code/problem.pddl', 1, 57)
