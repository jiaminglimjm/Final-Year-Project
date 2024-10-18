import pygame
import pymunk
import pymunk.pygame_util
from pygame.locals import *
import math

# Initialize Pygame and Pymunk
pygame.init()
screen_width, screen_height = 2000, 1000
screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption("Interactive Rod and Strings Physics Simulation with Pebbles")
clock = pygame.time.Clock()

# Pymunk space setup
space = pymunk.Space()
space.gravity = (0, 763)  # Gravity (pixels/sec^2)

# Scaling factors
PPM = 800 / 1.0  # Pixels per meter

# Collision types
L_SHAPE_COLLISION_TYPE = 1
PEBBLE_COLLISION_TYPE = 2

# Box parameters
box_height = 0.3  # meters
box_width = 1.0  # meters
box_pos = [box_width * PPM // 2 - 200, screen_height // 2]
box_rect = pygame.Rect(0, 0, box_width * PPM, box_height * PPM)
box_rect.center = box_pos

# Fix the box's vertical position
box_pos[1] = screen_height // 2

# Rod parameters
rod_length = 0.3 * PPM  # Convert meters to pixels
rod_mass = 0.5  # kg
rod_moment = pymunk.moment_for_segment(
    rod_mass, (-rod_length / 2, 0), (rod_length / 2, 0), 5
)

# Create the rod body and shape
rod_body = pymunk.Body(rod_mass, rod_moment)
rod_body.position = box_pos
rod_body.angular_damping = 0.0  # Disable built-in damping
rod_shape = pymunk.Segment(
    rod_body, (-rod_length / 2, 0), (rod_length / 2, 0), 5
)
space.add(rod_body, rod_shape)

# Strings (modeled as constraints)
rod_margin = 0.08 * PPM  # Margin from the ends of the rod

# Attachment points on the rod
left_rod_attach = (-rod_length / 2 + rod_margin, 0)
right_rod_attach = (rod_length / 2 - rod_margin, 0)

# Box corner anchors
left_anchor = (
    box_pos[0] - box_rect.width / 2,
    box_pos[1] - box_rect.height / 2,
)
right_anchor = (
    box_pos[0] + box_rect.width / 2,
    box_pos[1] - box_rect.height / 2,
)

# Create string constraints with initial distances
def calculate_distance(point_a, point_b):
    return math.hypot(point_a[0] - point_b[0], point_a[1] - point_b[1])

left_string_length = calculate_distance(
    rod_body.position + left_rod_attach, left_anchor
)
right_string_length = calculate_distance(
    rod_body.position + right_rod_attach, right_anchor
)

# Create string constraints
left_string = pymunk.constraints.PinJoint(
    rod_body, space.static_body, left_rod_attach, left_anchor
)
left_string.distance = left_string_length
right_string = pymunk.constraints.PinJoint(
    rod_body, space.static_body, right_rod_attach, right_anchor
)
right_string.distance = right_string_length
space.add(left_string, right_string)

# Floor that extends across the whole screen
floor_y = box_pos[1] + box_rect.height / 2 - 5
floor_body = pymunk.Body(body_type=pymunk.Body.STATIC)
floor_shape = pymunk.Segment(floor_body, (0, floor_y), (screen_width, floor_y), 5)
floor_shape.friction = 0.0
space.add(floor_body, floor_shape)

# Ceiling at the top of the box to prevent the rod from going above it
ceiling_y = box_pos[1] - box_rect.height / 2
ceiling_body = pymunk.Body(body_type=pymunk.Body.STATIC)
ceiling_shape = pymunk.Segment(
    ceiling_body, (0, ceiling_y), (screen_width, ceiling_y), 5
)
space.add(ceiling_body, ceiling_shape)


# Collision handler between L-shapes and pebbles
def l_shape_pebble_pre_solve(arbiter, space, data):
    rod_vertical_velocity = rod_body.velocity[1]
    if rod_vertical_velocity <= 0:
        return True  # Process collision
    else:
        return False  # Ignore collision

handler = space.add_collision_handler(L_SHAPE_COLLISION_TYPE, PEBBLE_COLLISION_TYPE)
handler.pre_solve = l_shape_pebble_pre_solve



# Pebble parameters
pebble_radius = 0.01 * PPM / 2  # 3 cm diameter, so radius is 1.5 cm
pebbles = []
num_pebbles = int(screen_width / (pebble_radius * 2))  # Number of pebbles to fill the floor


# Initialize pebble count
pebble_count = 0

# Create physical shapes for the L-shapes
num_L_shapes = 12
l_shapes = []



def reset_simulation():
    global rod_body, rod_shape, left_string, right_string, pebbles, pebble_count, t_loop, MOVE_STATE, l_shapes

    # Remove all dynamic bodies and shapes from the space
    for body in space.bodies[:]:
        if body.body_type == pymunk.Body.DYNAMIC:
            space.remove(body)
    for shape in space.shapes[:]:
        if hasattr(shape, 'body') and shape.body.body_type == pymunk.Body.DYNAMIC:
            space.remove(shape)
    for constraint in space.constraints[:]:
        space.remove(constraint)

    # Reset variables
    pebble_count = 0
    t_loop = 0
    MOVE_STATE = 101
    pebbles.clear()
    l_shapes.clear()

    # Reinitialize the rod, strings, pebbles, and L-shapes
    initialize_simulation_objects()

def initialize_simulation_objects():
    global box_pos, box_rect, rod_body, rod_shape, left_string, right_string, pebbles, l_shapes

    box_pos = [box_width * PPM // 2 - 200, screen_height // 2]
    box_rect.center = box_pos

    # Rod parameters (you can keep these as they are or redefine if needed)
    rod_body = pymunk.Body(rod_mass, rod_moment)
    rod_body.position = box_pos
    rod_body.angular_damping = 0.0  # Disable built-in damping
    rod_shape = pymunk.Segment(
        rod_body, (-rod_length / 2, 0), (rod_length / 2, 0), 5
    )
    space.add(rod_body, rod_shape)

    # Strings
    left_string = pymunk.constraints.PinJoint(
        rod_body, space.static_body, left_rod_attach, left_anchor
    )
    left_string.distance = left_string_length
    right_string = pymunk.constraints.PinJoint(
        rod_body, space.static_body, right_rod_attach, right_anchor
    )
    right_string.distance = right_string_length
    space.add(left_string, right_string)

    # Re-add the collision handler (if needed)
    handler = space.add_collision_handler(L_SHAPE_COLLISION_TYPE, PEBBLE_COLLISION_TYPE)
    handler.pre_solve = l_shape_pebble_pre_solve

    # Pebbles
    for i in range(num_pebbles):
        pebble_body = pymunk.Body(0.1, pymunk.moment_for_circle(0.1, 0, pebble_radius))
        x_position = i * pebble_radius * 2 + pebble_radius
        pebble_body.position = x_position, floor_y - pebble_radius - 5  # Slightly above the floor
        pebble_shape = pymunk.Circle(pebble_body, pebble_radius)
        pebble_shape.friction = 5
        pebble_shape.elasticity = 0.8
        pebble_shape.collision_type = PEBBLE_COLLISION_TYPE
        # pebble_shape.filter = pymunk.ShapeFilter(group=1)
        space.add(pebble_body, pebble_shape)
        pebbles.append((pebble_body, pebble_shape))

    # L-shapes
    for i in range(num_L_shapes):
        # Position along the rod
        t = i / (num_L_shapes)
        x = t * rod_length - rod_length / 2
        y = 0

        # Vertical segment of L-shape
        l_vertical = pymunk.Segment(
            rod_body,
            (x, y),
            (x, y + 15),  # Adjust length as needed
            2
        )
        l_vertical.sensor = False  # Enable collision
        l_vertical.friction = 0.5
        l_vertical.collision_type = L_SHAPE_COLLISION_TYPE
        space.add(l_vertical)
        l_shapes.append(l_vertical)

        # Horizontal segment of L-shape
        l_horizontal = pymunk.Segment(
            rod_body,
            (x, y + 15),
            (x + 15, y + 15),
            2
        )
        l_horizontal.sensor = False  # Enable collision
        l_horizontal.friction = 0.5
        l_horizontal.collision_type = L_SHAPE_COLLISION_TYPE
        space.add(l_horizontal)
        l_shapes.append(l_horizontal)



reset_simulation()


# Pygame drawing options
draw_options = pymunk.pygame_util.DrawOptions(screen)

# Font for stats display
font = pygame.font.SysFont("Arial", 16)

# Main loop
running = True
time = 0  # Time variable for functions


MOVE_STATE = 101 # init
t_loop = 0
while running:
    dt = clock.tick(60) / 1000.0  # Delta time in seconds
    time += dt
    t_loop += dt

    # Key press states
    keys = pygame.key.get_pressed()
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_r:
                reset_simulation()


    # Adjust string lengths while keys are held down
    adjustment_speed = 0.1 * PPM * dt  # Updated adjustment speed

    # Calculate the current positions
    left_rod_point = rod_body.local_to_world(left_rod_attach)
    right_rod_point = rod_body.local_to_world(right_rod_attach)

    # Calculate the actual current distances between anchors and rod points
    current_left_string_length = calculate_distance(
        left_anchor, left_rod_point
    )
    current_right_string_length = calculate_distance(
        right_anchor, right_rod_point
    )

    # Limit the rod's vertical position
    rod_top_limit = ceiling_y + 20  # Slightly below the ceiling
    rod_bottom_limit = floor_y - 5  # Slightly above the floor

    # Adjust left string length with limits
    if keys[K_q] and rod_body.position[1] > rod_top_limit:
        left_string.distance = max(
            10, left_string.distance - adjustment_speed
        )
    if keys[K_w]:
        left_string.distance += adjustment_speed

    # Adjust right string length with limits
    if keys[K_p] and rod_body.position[1] > rod_top_limit:
        right_string.distance = max(
            10, right_string.distance - adjustment_speed
        )
    if keys[K_o]:
        right_string.distance += adjustment_speed

    if MOVE_STATE == 101:
        if t_loop < 0.25:
            left_string.distance -= 0.1 * PPM * dt
            right_string.distance -= 0.1 * PPM * dt
        elif t_loop < 0.5:
            left_string.distance += 0.1 * PPM * dt
            right_string.distance -= 0.1 * PPM * dt
        else:
            t_loop = 0
            MOVE_STATE = 102
    if MOVE_STATE == 102:
        t1 = 0.78 * 2
        t2 = t1 + 1 * 2
        t3 = t2 + 0.78 * 2
        t4 = t3 + 0.5 * 2
        if t_loop < t1:
            left_string.distance += 0.05 * PPM * dt
            right_string.distance += 0.05 * PPM * dt
        elif t_loop < t2:
            left_string.distance -= 0.05 * PPM * dt
            right_string.distance += 0.05 * PPM * dt
        elif t_loop < t3:
            left_string.distance -= 0.05 * PPM * dt
            right_string.distance -= 0.05 * PPM * dt
        elif t_loop < t4:
            left_string.distance += 0.1 * PPM * dt
            right_string.distance -= 0.1 * PPM * dt
        else:
            t_loop = 0

    # Move the box with arrow keys (horizontal movement only)
    box_movement_speed = 60 * dt  # Pixels per second
    if keys[K_LEFT]:
        box_pos[0] -= box_movement_speed
    if keys[K_RIGHT]:
        box_pos[0] += box_movement_speed
    box_rect.centerx = box_pos[0]  # Update only the x-coordinate

    # Update anchors based on new box position
    left_anchor = (
        box_pos[0] - box_rect.width / 2,
        box_pos[1] - box_rect.height / 2,
    )
    right_anchor = (
        box_pos[0] + box_rect.width / 2,
        box_pos[1] - box_rect.height / 2,
    )
    left_string.anchor_b = left_anchor
    right_string.anchor_b = right_anchor

    # Apply manual rotational damping
    damping_coefficient = 3.0  # Adjust this value for stronger or weaker damping
    rod_body.angular_velocity *= math.exp(-damping_coefficient * dt)

    # Limit the rod's vertical position
    if rod_body.position[1] < rod_top_limit:
        #rod_body.position[1] = rod_top_limit
        rod_body.velocity = (rod_body.velocity[0], max(0, rod_body.velocity[1]))
    if rod_body.position[1] > rod_bottom_limit:
        #rod_body.position[1] = rod_bottom_limit
        rod_body.velocity = (rod_body.velocity[0], min(0, rod_body.velocity[1]))

    # Step physics
    space.step(dt)

    # Check pebbles and remove if they reach 3/4 of the box height
    pebble_removal_height = box_pos[1] - box_rect.height * 0.25
    for pebble_body, pebble_shape in pebbles[:]:
        if pebble_body.position[1] <= pebble_removal_height:
            # Remove pebble from space and list
            space.remove(pebble_body, pebble_shape)
            pebbles.remove((pebble_body, pebble_shape))
            pebble_count += 1

    # Clear screen
    screen.fill((255, 255, 255))

    # Draw box top border
    top_left = (
        box_pos[0] - box_rect.width / 2,
        box_pos[1] - box_rect.height / 2,
    )
    top_right = (
        box_pos[0] + box_rect.width / 2,
        box_pos[1] - box_rect.height / 2,
    )
    pygame.draw.line(screen, (200, 200, 200), top_left, top_right, 2)

    # Draw floor
    pygame.draw.line(
        screen, (0, 0, 0), (0, floor_y), (screen_width, floor_y), 5
    )

    # Draw pebbles
    for pebble_body, pebble_shape in pebbles:
        x, y = pebble_body.position
        pygame.draw.circle(screen, (80, 80, 60), (int(x), int(y)), int(pebble_radius))

    # Draw strings
    pygame.draw.line(screen, (0, 0, 0), left_anchor, left_rod_point, 2)
    pygame.draw.line(screen, (0, 0, 0), right_anchor, right_rod_point, 2)

    # Draw arrows showing the magnitude of tension in the strings
    # Define the maximum arrow length (in pixels)
    max_arrow_length = 500  # Adjust as needed for visual clarity

    # Define the tension range in Newtons
    tension_min = 200
    tension_max = 10000

    # String tensions (approximated from the constraint impulses)
    left_tension = left_string.impulse / dt if dt > 0 else 0
    right_tension = right_string.impulse / dt if dt > 0 else 0

    # Function to map tension to arrow length
    def map_tension_to_length(tension):
        # Clamp the tension within the specified range
        tension = max(tension_min, min(tension, tension_max))
        # Map the tension to arrow length proportionally
        return ((tension - tension_min) / (tension_max - tension_min)) * max_arrow_length

    # Left arrow
    left_arrow_length = map_tension_to_length(left_tension)
    left_dx = left_anchor[0] - left_rod_point[0]
    left_dy = left_anchor[1] - left_rod_point[1]
    left_distance = math.hypot(left_dx, left_dy)
    left_unit_dx = left_dx / left_distance
    left_unit_dy = left_dy / left_distance
    left_arrow_start = left_rod_point
    left_arrow_end = (
        left_arrow_start[0] + left_unit_dx * left_arrow_length,
        left_arrow_start[1] + left_unit_dy * left_arrow_length,
    )
    pygame.draw.line(screen, (255, 0, 0), left_arrow_start, left_arrow_end, 3)
    # Left arrowhead
    arrowhead_size = 10
    left_arrowhead_point1 = (
        left_arrow_end[0] - left_unit_dx * arrowhead_size - left_unit_dy * arrowhead_size,
        left_arrow_end[1] - left_unit_dy * arrowhead_size + left_unit_dx * arrowhead_size,
    )
    left_arrowhead_point2 = (
        left_arrow_end[0] - left_unit_dx * arrowhead_size + left_unit_dy * arrowhead_size,
        left_arrow_end[1] - left_unit_dy * arrowhead_size - left_unit_dx * arrowhead_size,
    )
    pygame.draw.polygon(
        screen, (255, 0, 0), [left_arrow_end, left_arrowhead_point1, left_arrowhead_point2]
    )

    # Right arrow
    right_arrow_length = map_tension_to_length(right_tension)
    right_dx = right_anchor[0] - right_rod_point[0]
    right_dy = right_anchor[1] - right_rod_point[1]
    right_distance = math.hypot(right_dx, right_dy)
    right_unit_dx = right_dx / right_distance
    right_unit_dy = right_dy / right_distance
    right_arrow_start = right_rod_point
    right_arrow_end = (
        right_arrow_start[0] + right_unit_dx * right_arrow_length,
        right_arrow_start[1] + right_unit_dy * right_arrow_length,
    )
    pygame.draw.line(screen, (255, 0, 0), right_arrow_start, right_arrow_end, 3)
    # Right arrowhead
    right_arrowhead_point1 = (
        right_arrow_end[0] - right_unit_dx * arrowhead_size - right_unit_dy * arrowhead_size,
        right_arrow_end[1] - right_unit_dy * arrowhead_size + right_unit_dx * arrowhead_size,
    )
    right_arrowhead_point2 = (
        right_arrow_end[0] - right_unit_dx * arrowhead_size + right_unit_dy * arrowhead_size,
        right_arrow_end[1] - right_unit_dy * arrowhead_size - right_unit_dx * arrowhead_size,
    )
    pygame.draw.polygon(
        screen,
        (255, 0, 0),
        [right_arrow_end, right_arrowhead_point1, right_arrowhead_point2],
    )

    # Draw rod
    rod_start = rod_body.local_to_world((-rod_length / 2, 0))
    rod_end = rod_body.local_to_world((rod_length / 2, 0))
    pygame.draw.line(screen, (0, 0, 255), rod_start, rod_end, 5)

    # Draw L-shapes attached to the rod
    for l_shape in l_shapes:
        start = rod_body.local_to_world(l_shape.a)
        end = rod_body.local_to_world(l_shape.b)
        pygame.draw.line(screen, (0, 0, 0), start, end, 2)

    # Rod position and angle
    rod_x, rod_y = rod_body.position
    rod_angle = math.degrees(rod_body.angle) % 360

    # Display live stats
    stats_text = [
        f"Left String Length: {left_string.distance / PPM:.2f} m",
        f"Right String Length: {right_string.distance / PPM:.2f} m",
        f"Left String Tension: {left_tension:.2f} N",
        f"Right String Tension: {right_tension:.2f} N",
        f"Rod Position: ({rod_x / PPM:.2f}, {rod_y / PPM:.2f}) m",
        f"Rod Angle: {rod_angle:.2f}°",
        f"Time: {time:.3f}"
    ]
    for i, text in enumerate(stats_text):
        render = font.render(text, True, (0, 0, 0))
        screen.blit(render, (10, 10 + i * 20))

    # Display pebble count
    pebble_text = f"Pebble Count: {pebble_count}"
    pebble_render = font.render(pebble_text, True, (0, 0, 0))
    screen.blit(pebble_render, (10, 10 + (len(stats_text) + 1) * 20))

    # Update display
    pygame.display.flip()

pygame.quit()
