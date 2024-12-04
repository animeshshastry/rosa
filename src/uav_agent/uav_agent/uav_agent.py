#!/usr/bin/env python3.10
#  Copyright (c) 2024. Jet Propulsion Laboratory. All rights reserved.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#  https://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import asyncio
import os
from datetime import datetime

import dotenv
import pyinputplus as pyip

import rclpy
from rclpy.node import Node
from rclpy import qos
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from langchain.agents import tool, Tool
# from langchain_ollama import ChatOllama
from rich.console import Console
from rich.console import Group
from rich.live import Live
from rich.markdown import Markdown
from rich.panel import Panel
from rich.text import Text
from rosa import ROSA

from uav_agent.tools import uav as uav_tools
from uav_agent.help import get_help
from uav_agent.llm import get_llm
from uav_agent.prompts import get_prompts

class RobotAgentNode(Node):
    def __init__(self):
        super().__init__('uav_agent_node')

        # # Add the default robot publisher on startup
        # qos_profile = QoSProfile(
        #     reliability=QoSReliabilityPolicy.RELIABLE,
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     depth=10
        # )
        # self.cmd_publisher = self.create_publisher(msg_type=Twist, topic="/alfa/cmd_vel", qos_profile=qos_profile, callback_group=MutuallyExclusiveCallbackGroup())
        self.cmd_publisher = self.create_publisher(Twist, "/alfa/cmd_vel", 10)
        uav_tools.add_cmd_vel_pub("alfa", self.cmd_publisher)

        self.cmd_publisher = self.create_publisher(Twist, "/warty/cmd_vel", 10)
        uav_tools.add_cmd_vel_pub("warty", self.cmd_publisher)

        # self.pose_subscriber = self.create_subscription(PoseWithCovarianceStamped,'/alfa/openvins/poseimu', uav_tools.pose_callback, 10)
        self.pose_subscriber = self.create_subscription(Odometry,'/alfa/raw_odom', uav_tools.pose_callback, 10)

class RobotAgent(ROSA):

    def __init__(self, verbose: bool = True):

        # self.declare_parameter('streaming', 'True')
        streaming = True

        self.__blacklist = ["master", "docker"]
        self.__prompts = get_prompts()
        self.__llm = get_llm(streaming=streaming)
        self.__streaming = streaming

        # # Another method for adding tools
        # blast_off = Tool(
        #     name="blast_off",
        #     func=self.blast_off,
        #     description="Make the turtle blast off!",
        # )

        super().__init__(
            ros_version=2,
            llm=self.__llm,
            tools=[],
            tool_packages=[uav_tools],
            blacklist=self.__blacklist,
            prompts=self.__prompts,
            verbose=verbose,
            accumulate_chat_history=True,
            streaming=streaming,
        )

        self.examples = [
            "Show me how to move the uav forward.",
            "Draw a 5-point star using the uav.",
            "Move to (10, 10) and draw a small hexagon.",
            "Give me a list of nodes, topics, services, params, and log files.",
        ]

        self.command_handler = {
            "help": lambda: self.submit(get_help(self.examples)),
            "examples": lambda: self.submit(self.choose_example()),
            "clear": lambda: self.clear(),
        }


    # def blast_off(self, input: str):
    #     return f"""
    #     Ok, we're blasting off at the speed of light!

    #     <ROSA_INSTRUCTIONS>
    #         You should now use your tools to make the robot move around the screen at high speeds.
    #     </ROSA_INSTRUCTIONS>
    #     """

    @property
    def greeting(self):
        greeting = Text(
            "\nHi! I'm the ROSA-UAV agent 🐢🤖. How can I help you today?\n"
        )
        greeting.stylize("frame bold blue")
        greeting.append(
            f"Try {', '.join(self.command_handler.keys())} or exit.",
            style="italic",
        )
        return greeting

    def choose_example(self):
        """Get user selection from the list of examples."""
        return pyip.inputMenu(
            self.examples,
            prompt="\nEnter your choice and press enter: \n",
            numbered=True,
            blank=False,
            timeout=60,
            default="1",
        )

    async def clear(self):
        """Clear the chat history."""
        self.clear_chat()
        self.last_events = []
        self.command_handler.pop("info", None)
        os.system("clear")

    def get_input(self, prompt: str):
        """Get user input from the console."""
        return pyip.inputStr(prompt, default="help")

    async def run(self):
        """
        Run the UAVAgent's main interaction loop.

        This method initializes the console interface and enters a continuous loop to handle user input.
        It processes various commands including 'help', 'examples', 'clear', and 'exit', as well as
        custom user queries. The method uses asynchronous operations to stream responses and maintain
        a responsive interface.

        The loop continues until the user inputs 'exit'.

        Returns:
            None

        Raises:
            Any exceptions that might occur during the execution of user commands or streaming responses.
        """
        await self.clear()
        console = Console()

        while True:
            console.print(self.greeting)
            input = self.get_input("> ")

            # Handle special commands
            if input == "exit":
                break
            elif input in self.command_handler:
                await self.command_handler[input]()
            else:
                await self.submit(input)

    async def submit(self, query: str):
        if self.__streaming:
            await self.stream_response(query)
        else:
            self.print_response(query)

    def print_response(self, query: str):
        """
        Submit the query to the agent and print the response to the console.

        Args:
            query (str): The input query to process.

        Returns:
            None
        """
        response = self.invoke(query)
        console = Console()
        content_panel = None

        with Live(
            console=console, auto_refresh=True, vertical_overflow="visible"
        ) as live:
            content_panel = Panel(
                Markdown(response), title="Final Response", border_style="green"
            )
            live.update(content_panel, refresh=True)

    async def stream_response(self, query: str):
        """
        Stream the agent's response with rich formatting.

        This method processes the agent's response in real-time, updating the console
        with formatted output for tokens and keeping track of events.

        Args:
            query (str): The input query to process.

        Returns:
            None

        Raises:
            Any exceptions raised during the streaming process.
        """
        console = Console()
        content = ""
        self.last_events = []

        panel = Panel("", title="Streaming Response", border_style="green")

        with Live(panel, console=console, auto_refresh=False) as live:
            async for event in self.astream(query):
                event["timestamp"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[
                    :-3
                ]
                if event["type"] == "token":
                    content += event["content"]
                    panel.renderable = Markdown(content)
                    live.refresh()
                elif event["type"] in ["tool_start", "tool_end", "error"]:
                    self.last_events.append(event)
                elif event["type"] == "final":
                    content = event["content"]
                    if self.last_events:
                        panel.renderable = Markdown(
                            content
                            + "\n\nType 'info' for details on how I got my answer."
                        )
                    else:
                        panel.renderable = Markdown(content)
                    panel.title = "Final Response"
                    live.refresh()

        if self.last_events:
            self.command_handler["info"] = self.show_event_details
        else:
            self.command_handler.pop("info", None)

    async def show_event_details(self):
        """
        Display detailed information about the events that occurred during the last query.
        """
        console = Console()

        if not self.last_events:
            console.print("[yellow]No events to display.[/yellow]")
            return
        else:
            console.print(Markdown("# Tool Usage and Events"))

        for event in self.last_events:
            timestamp = event["timestamp"]
            if event["type"] == "tool_start":
                console.print(
                    Panel(
                        Group(
                            Text(f"Input: {event.get('input', 'None')}"),
                            Text(f"Timestamp: {timestamp}", style="dim"),
                        ),
                        title=f"Tool Started: {event['name']}",
                        border_style="blue",
                    )
                )
            elif event["type"] == "tool_end":
                console.print(
                    Panel(
                        Group(
                            Text(f"Output: {event.get('output', 'N/A')}"),
                            Text(f"Timestamp: {timestamp}", style="dim"),
                        ),
                        title=f"Tool Completed: {event['name']}",
                        border_style="green",
                    )
                )
            elif event["type"] == "error":
                console.print(
                    Panel(
                        Group(
                            Text(f"Error: {event['content']}", style="bold red"),
                            Text(f"Timestamp: {timestamp}", style="dim"),
                        ),
                        border_style="red",
                    )
                )
            console.print()

        console.print("[bold]End of events[/bold]\n")


async def ros_loop(node):
    # rclpy.spin(node)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        await asyncio.sleep(0.05)

def main():
    rclpy.init()
    robot_agent_node = RobotAgentNode()

    dotenv.load_dotenv(dotenv.find_dotenv())
    # streaming = rclpy.get_param("~streaming", False)
    robot_agent = RobotAgent(verbose=True)
    # asyncio.run(robot_agent.run())

    # future = asyncio.wait([robot_agent.run(), ros_loop(robot_agent_node)])
    # asyncio.get_event_loop().run_until_complete(future)

    loop = asyncio.get_event_loop()
    loop.create_task(robot_agent.run())
    loop.create_task(ros_loop(robot_agent_node))
    loop.create_task(uav_tools.mission_loop())
    loop.run_forever()

if __name__ == "__main__":
    main()
