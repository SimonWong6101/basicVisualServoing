#include <cstdlib>
#include <iostream>
#include <visp3/core/vpUDPClient.h>
#include <chrono>
#include <thread>
// Starts an UDP session from localhost, just printing whatever text is sent to it
int main() {
	try {
		std::string servername = "127.0.0.1";
		unsigned int port = 50037;
		vpUDPClient client(servername, port);
		while (true) {
			std::cout << "Enter the message to send:" << std::endl;
			std::string msg = "";
			std::getline(std::cin, msg);
			if (client.send(msg) != (int)msg.size())
				std::cerr << "Error client.send()!" << std::endl;
			if (client.receive(msg))
				std::cout << "Receive from the server: " << msg << std::endl;
			while (true)
			{
				std::this_thread::sleep_for(std::chrono::microseconds(4));
				if (client.receive(msg))
					std::cout << "Receive from the server: " << msg << std::endl;
			}
		}
		return EXIT_SUCCESS;
	}
	catch (const vpException &e) {
		std::cerr << "Catch an exception: " << e.what() << std::endl;
		return EXIT_FAILURE;
	}
}