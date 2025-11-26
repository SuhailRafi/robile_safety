### Implement the safety functionalities for the Robile by implementing all
### required behaviours here. Feel free to define additional behaviours if necessary

import rclpy
import py_trees as pt
import py_trees_ros as ptr
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math
import time

class Rotate(pt.behaviour.Behaviour):
    """Rotates the robot about the z-axis 
    """
    def __init__(self, name="rotate platform",
                 topic_name="/cmd_vel",
                 ang_vel=1.0):
        super(Rotate, self).__init__(name)

        # TODO: initialise any necessary class variables
        # YOUR CODE HERE
        self.topic_name = topic_name
        self.ang_vel = ang_vel
        self.node = None
        self.publisher = None

    def setup(self, **kwargs):
        """Setting up things which generally might require time to prevent delay in the tree initialisation
        """
        self.logger.info("🔄 [ROTATE] setting up rotate behaviour")
        
        try: self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e 

        # TODO: setup any necessary publishers or subscribers
        # YOUR CODE HERE
        self.publisher = self.node.create_publisher(Twist, self.topic_name, 10)

        return True

    def update(self):
        """Rotates the robot at the maximum allowed angular velocity.
        Note: The actual behaviour is implemented here.

        """
        self.logger.info("🔄 [ROTATE] Robot rotating due to low battery")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        # TODO: implement the primary function of the behaviour and decide which status to return 
        # based on the structure of your behaviour tree

        # Hint: to return a status, for example, SUCCESS, pt.common.Status.SUCCESS can be used

        # YOUR CODE HERE
        twist_msg = Twist()
        twist_msg.angular.z = self.ang_vel
        self.publisher.publish(twist_msg)
        
        return pt.common.Status.RUNNING

    def terminate(self, new_status):
        """Trigerred once the execution of the behaviour finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info("🔄 [ROTATE] terminate: publishing zero angular velocity")

        # TODO: implement the termination of the behaviour, i.e. what should happen when the behaviour 
        # finishes its execution

        # YOUR CODE HERE
        twist_msg = Twist()
        twist_msg.angular.z = 0.0
        self.publisher.publish(twist_msg)

        return super().terminate(new_status)

class StopMotion(pt.behaviour.Behaviour):
    """Stops the robot when it is controlled using a joystick or with a cmd_vel command
    """
    
    # TODO: Implement a behaviour to stop the robot's motion

    # YOUR CODE HERE
    def __init__(self, name="stop motion", topic_name="/cmd_vel"):
        super(StopMotion, self).__init__(name)
        self.topic_name = topic_name
        self.node = None
        self.publisher = None

    def setup(self, **kwargs):
        self.logger.info("🛑 [STOP MOTION] setting up stop motion behaviour")
        
        try: self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}]".format(self.qualified_name)
            raise KeyError(error_message) from e 

        self.publisher = self.node.create_publisher(Twist, self.topic_name, 10)
        return True

    def update(self):
        self.logger.info("🛑 [STOP MOTION] EMERGENCY STOP - Obstacle detected!")
        
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        self.publisher.publish(twist_msg)
        
        return pt.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.info("🛑 [STOP MOTION] terminate: behaviour finished")
        return super().terminate(new_status)

class BatteryStatus2bb(ptr.subscribers.ToBlackboard):
    """Checks the battery status
    """
    def __init__(self, battery_voltage_topic_name: str="/battery_voltage",
                 name: str='Battery2BB',
                 threshold: float=30.0):
        super().__init__(name=name,
                         topic_name=battery_voltage_topic_name,
                         topic_type=Float32,
                         blackboard_variables={'battery': 'data'},
                         initialise_variables={'battery': 100.0},
                         clearing_policy=pt.common.ClearingPolicy.NEVER,  # to decide when data should be cleared/reset.
                         qos_profile=ptr.utilities.qos_profile_unlatched())
        self.blackboard.register_key(key='battery_low_warning', access=pt.common.Access.WRITE)

        # YOUR CODE HERE
        self.threshold = threshold
        self.blackboard.battery_low_warning = False

    def update(self):
        """Calls the parent to write the raw data to the blackboard and then check against the
        threshold to determine if a low warning flag should also be updated.
        """
        self.logger.info('🔋 [BATTERY] update: running battery_status2bb update')
        self.logger.debug("%s.update()" % self.__class__.__name__)
        
        """check battery voltage level stored in self.blackboard.battery. By comparing with 
        threshold value, update the value of self.blackboad.battery_low_warning
        """

        # TODO: based on the battery voltage level, update the value of self.blackboard.battery_low_warning
        # and return the status of the behaviour based on your logic of the behaviour tree

        # YOUR CODE HERE
        status = super().update()
        
        if status != pt.common.Status.RUNNING:
            battery_level = self.blackboard.battery
            self.blackboard.battery_low_warning = battery_level < self.threshold
            
            if self.blackboard.battery_low_warning: self.logger.warning(f"🔋 [BATTERY] LOW BATTERY: {battery_level}% < {self.threshold}%")
            else: self.logger.info(f"🔋 [BATTERY] OK: {battery_level}%")
                
            return pt.common.Status.SUCCESS
        return status

class LaserScan2bb(ptr.subscribers.ToBlackboard):
    """Checks the laser scan measurements to avoid possible collisions.
    """
    def __init__(self, topic_name: str="/scan",
                 name: str='Scan2BB',
                 safe_range: float=1.00):
        super().__init__(name=name,
                         topic_name=topic_name,
                         topic_type=LaserScan,
                         blackboard_variables={'laser_scan':'ranges'},
                         clearing_policy=pt.common.ClearingPolicy.NEVER,  # to decide when data should be cleared/reset.
                         qos_profile=QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                                history=QoSHistoryPolicy.KEEP_LAST,
                                                depth=10))
        
        # TODO: initialise class variables and blackboard variables
        # YOUR CODE HERE
        self.safe_range = safe_range
        self.blackboard.register_key(key='collision_warning', access=pt.common.Access.WRITE)
        self.blackboard.register_key(key='min_distance', access=pt.common.Access.WRITE)
        self.blackboard.collision_warning = False
        self.blackboard.min_distance = float('inf')

    def update(self):
        # TODO: impletment the update function to check the laser scan data and update the blackboard variable
        # YOUR CODE HERE
        status = super().update()
        
        if status != pt.common.Status.RUNNING:
            laser_data = self.blackboard.laser_scan
            
            if laser_data is not None:
                min_distance = float('inf')
                for distance in laser_data:
                    if not math.isinf(distance) and not math.isnan(distance) and distance > 0: min_distance = min(min_distance, distance)
                
                # If we found valid distances, check for collision
                if min_distance != float('inf'):
                    self.blackboard.min_distance = min_distance
                    self.blackboard.collision_warning = min_distance < self.safe_range
                    
                    if self.blackboard.collision_warning: self.logger.warning(f"🚨 [LASER] COLLISION WARNING: {min_distance:.2f}m < {self.safe_range}m")
                    else: self.logger.info(f"📏 [LASER] Clear: {min_distance:.2f}m")
                else:
                    self.blackboard.min_distance = float('inf')
                    self.blackboard.collision_warning = False
            
            return pt.common.Status.SUCCESS
        return status
    

### Implement a behaviour tree using your previously implemented behaviours here

import py_trees as pt
import py_trees_ros as ptr
import operator

import py_trees.console as console
import rclpy
import sys
import time

class StateMonitor:
    """Monitors and logs behaviour tree state changes with clean output"""
    
    def __init__(self):
        self.previous_state = "UNKNOWN"
        self.current_state = "UNKNOWN"
        self.tick_count = 0
        self.state_history = []
        self.start_time = time.time()
        
        # Create a blackboard client to access the shared blackboard
        self.blackboard = pt.blackboard.Client(name="StateMonitor")
        self.blackboard.register_key(key='battery', access=pt.common.Access.READ)
        self.blackboard.register_key(key='battery_low_warning', access=pt.common.Access.READ)
        self.blackboard.register_key(key='collision_warning', access=pt.common.Access.READ)
        self.blackboard.register_key(key='min_distance', access=pt.common.Access.READ)
        
    def determine_state(self):
        """Determine current state by checking blackboard variables"""
        try:
            if self.blackboard.collision_warning: return "🚨 COLLISION_AVOIDANCE"
            elif self.blackboard.battery_low_warning: return "🔋 BATTERY_CHARGING"
            else: return "🟢 NORMAL_OPERATION"
        except: return "❓ UNKNOWN"
    
    def get_state_color(self, state):
        """Return color codes for different states"""
        colors = {
            "🚨 COLLISION_AVOIDANCE": "\033[91m",  # Red
            "🔋 BATTERY_CHARGING": "\033[93m",     # Yellow  
            "🟢 NORMAL_OPERATION": "\033[92m",     # Green
            "❓ UNKNOWN": "\033[90m"               # Gray
        }
        return colors.get(state, "\033[0m")
    
    def log_state_info(self):
        """Log current state and detect changes with clean output"""
        self.current_state = self.determine_state()
        
        # Check for state change
        if self.current_state != self.previous_state:
            self.print_state_change()
            
            # Record state change
            self.state_history.append({
                'tick': self.tick_count,
                'time': time.time() - self.start_time,
                'from_state': self.previous_state,
                'to_state': self.current_state
            })
        
        # Always show current state in a clean way
        self.print_current_state()
        
        self.previous_state = self.current_state
        self.tick_count += 1
    
    def print_state_change(self):
        """Print state change in a clear, noticeable way"""
        print("\n" + "═" * 60)
        print("🎯 STATE CHANGE DETECTED!")
        print("═" * 60)
        color_from = self.get_state_color(self.previous_state)
        color_to = self.get_state_color(self.current_state)
        reset = "\033[0m"
        
        print(f"   FROM: {color_from}{self.previous_state}{reset}")
        print(f"   TO:   {color_to}{self.current_state}{reset}")
        print(f"   TIME: {time.time() - self.start_time:.1f}s | TICK: {self.tick_count}")
        print("═" * 60)
    
    def print_current_state(self):
        """Print current state in a clean, always-visible format"""
        try:
            battery_level = self.blackboard.battery
            battery_low = self.blackboard.battery_low_warning
            collision_warning = self.blackboard.collision_warning
            min_distance = getattr(self.blackboard, 'min_distance', 'N/A')
            
            # Create status indicators
            battery_icon = "🔴" if battery_low else "🟢"
            
            # Collision icon based on distance
            if min_distance == 'N/A': collision_icon = "⚪"
            elif collision_warning: collision_icon = "🔴"
            else: collision_icon = "🟢"
            
            # Clear line and print current state
            distance_str = f"{min_distance:.2f}m" if isinstance(min_distance, float) else min_distance
            print(f"\r📊 CURRENT STATE: {self.current_state} | "
                  f"Battery: {battery_icon} {battery_level:.1f}% | "
                  f"Obstacle: {collision_icon} {distance_str} | "
                  f"Time: {time.time() - self.start_time:.1f}s | "
                  f"Tick: {self.tick_count}", end="", flush=True)
                  
        except Exception as e:
            print(f"\r📊 CURRENT STATE: {self.current_state} | "
                  f"Data: ❌ | "
                  f"Time: {time.time() - self.start_time:.1f}s | "
                  f"Tick: {self.tick_count}", end="", flush=True)
    
    def print_state_history(self):
        """Print history of state changes"""
        if not self.state_history:
            print("\n\n📋 No state changes recorded yet.")
            return
            
        print("\n\n" + "📋 STATE HISTORY " + "═" * 44)
        for change in self.state_history[-10:]:  # Last 10 changes
            color_from = self.get_state_color(change['from_state'])
            color_to = self.get_state_color(change['to_state'])
            reset = "\033[0m"
            
            print(f"   ⏱️  {change['time']:5.1f}s | "
                  f"🔄 {color_from}{change['from_state']:20}{reset} → "
                  f"{color_to}{change['to_state']}{reset}")

def create_root() -> pt.behaviour.Behaviour:
    """Structures a behaviour tree to monitor the battery status, and start
    to rotate if the battery is low and stop if it detects an obstacle in front of it.
    """

    # we define the root node
    root = pt.composites.Parallel(name="root",
                                  policy=pt.common.ParallelPolicy.SuccessOnAll(synchronise=False))    

    ### we create a sequence node called "Topics2BB" and a selector node called "Priorities"
    topics2BB = pt.composites.Sequence("Topics2BB", memory=False)
    priorities = pt.composites.Selector("Priorities", memory=False)

    ### we create an "Idle" node, which is a running node to keep the robot idle
    idle = pt.behaviours.Running(name="Idle")
    
    """
    TODO:  The first and second level of the tree structure is defined above, but please
    define the rest of the tree structure.

    Class definitions for your behaviours are provided in behaviours.py; you also need to fill out
    the behaviour implementations!

    HINT: Some behaviours from pt.behaviours may be useful to use as well.
    """

    # YOUR CODE HERE
    # Create battery monitoring branch
    battery_monitor = pt.composites.Sequence("BatteryMonitor", memory=False)
    battery_check = BatteryStatus2bb(name="CheckBattery", threshold=30.0)
    rotate_action = Rotate(name="RotateOnLowBattery", ang_vel=1.0)
    battery_monitor.add_children([battery_check, rotate_action])

    # Create collision avoidance branch
    collision_monitor = pt.composites.Sequence("CollisionMonitor", memory=False)
    laser_check = LaserScan2bb(name="CheckLaser", safe_range=1.00)
    stop_action = StopMotion(name="StopOnCollision")
    collision_monitor.add_children([laser_check, stop_action])

    # Create condition checks for the priorities
    battery_condition = pt.behaviours.CheckBlackboardVariableValue(
        name="BatteryLow?",
        check=pt.common.ComparisonExpression(
            variable="battery_low_warning",
            value=True,
            operator=operator.eq
        )
    )

    collision_condition = pt.behaviours.CheckBlackboardVariableValue(
        name="CollisionDetected?",
        check=pt.common.ComparisonExpression(
            variable="collision_warning",
            value=True,
            operator=operator.eq
        )
    )

    # Create priority branches
    collision_branch = pt.composites.Sequence("CollisionPriority", memory=False)
    collision_branch.add_children([collision_condition, collision_monitor])

    battery_branch = pt.composites.Sequence("BatteryPriority", memory=False)
    battery_branch.add_children([battery_condition, battery_monitor])

    # TODO: construct the behaviour tree structure using the nodes and behaviours defined above
    # HINT: for reference, the sample tree structure in the README.md file might be useful

    root.add_children([topics2BB, priorities])

    # YOUR CODE HERE
    # Add battery and laser topics to Topics2BB
    topics2BB.add_children([BatteryStatus2bb(name="Battery2BB"), LaserScan2bb(name="Laser2BB")])
    
    # Build priorities selector (collision has highest priority, then battery, then idle)
    priorities.add_children([collision_branch, battery_branch, idle])

    return root

def main():
    """Initialises and executes the behaviour tree
    """
    rclpy.init(args=None)

    # Create state monitor
    state_monitor = StateMonitor()
    
    # Print initial information
    print("\n" + "🚀 BEHAVIOUR TREE STARTED " + "═" * 35)
    print("   Real-time State Monitoring Active")
    print("   State changes will be highlighted below")
    print("   Current state is always visible at bottom")
    print("═" * 60)
    print("\n📋 AVAILABLE STATES:")
    print("   🚨 COLLISION_AVOIDANCE - Emergency stop (obstacle detected)")
    print("   🔋 BATTERY_CHARGING    - Rotating in place (low battery)")
    print("   🟢 NORMAL_OPERATION    - Standard operation")
    print("\n" + "═" * 60)
    print("Starting monitoring...\n")

    root = create_root()
    tree = ptr.trees.BehaviourTree(root=root, unicode_tree_debug=True)

    try: tree.setup(timeout=30.0)
    except ptr.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    
    # frequency of ticks
    tree.tick_tock(period_ms=100)    

    try:
        while rclpy.ok():
            # Monitor and log state
            state_monitor.log_state_info()
            rclpy.spin_once(tree.node, timeout_sec=0.1)
            
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        print("\n\n" + "🛑 BEHAVIOUR TREE STOPPED " + "═" * 35)
        state_monitor.print_state_history()
        print("\n" + "═" * 60)
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__': main()