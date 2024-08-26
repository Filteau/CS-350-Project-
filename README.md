# CS-350-Project-
CS 350 Project One: Thermostat

#Project Summary
In this course, I worked on developing a prototype for a smart thermostat using the TI CC3220SF microcontroller. The project aimed to solve the problem of reading room temperature, adjusting the temperature settings, and controlling an LED to simulate a heating system, all while sending the temperature data over UART as if it were being transmitted to a server. The ultimate goal was to create a low-level prototype that could later be expanded to connect to the cloud and integrate with SysTec's server software.

#Achievements
I am particularly proud of how I successfully integrated multiple hardware components—such as the TMP006 temperature sensor, LEDs, and buttons—using different communication protocols like I2C and GPIO. My ability to create a reliable task scheduler that managed these components effectively demonstrated my capability in embedded systems development. I was able to make the system responsive and efficient, which is crucial for real-time embedded applications.

#Areas for Improvement
While I was able to meet the project requirements, I believe there is room for improvement in optimizing the task scheduler. While the scheduler works, I could refine it further for better performance and more efficient use of the microcontroller's resources. Additionally, I could enhance the error handling mechanisms, especially when dealing with potential communication failures on the I2C bus.

#Tools and Resources
Throughout the project, I added several tools to my support network, including the TI Code Composer Studio for development and debugging, as well as GitHub for version control and project management. I also utilized online resources like the TI documentation, community forums, and technical articles on embedded systems to aid in the development process.

#Transferable Skills
The skills I developed during this project, such as configuring peripherals, managing communication protocols, and implementing a task scheduler, are highly transferable to other embedded systems projects. Additionally, my experience with using interrupts and real-time control will be beneficial in future coursework and professional projects that require low-level hardware interaction.

#Project Maintenance and Adaptability
To ensure that the project is maintainable, readable, and adaptable, I followed coding best practices, including writing clear comments, using meaningful variable names, and structuring the code into modular functions. The project is documented thoroughly, and the code is organized in a way that future developers can easily understand and build upon it. Additionally, the system is designed with scalability in mind, allowing for easy integration of additional features like cloud connectivity.
