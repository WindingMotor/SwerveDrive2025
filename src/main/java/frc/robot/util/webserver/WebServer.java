// Copyright (c) 2024 - 2025 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.webserver;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;

public class WebServer {
	private HttpServer server;
	private final Map<String, Command> commandMap = new HashMap<>();
	private final String deployDirectory = Filesystem.getDeployDirectory().getAbsolutePath();

	public WebServer() {
		try {
			server = HttpServer.create(new InetSocketAddress(5800), 0);
			server.createContext("/", new StaticFileHandler());
			server.createContext("/command", new CommandHandler());
			server.createContext("/status", new StatusHandler()); // Add status endpoint
			server.setExecutor(null);
			server.start();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public void registerCommand(String id, Command command) {
		commandMap.put(id, command);
	}

	class StaticFileHandler implements HttpHandler {
		@Override
		public void handle(HttpExchange exchange) throws IOException {
			String path = exchange.getRequestURI().getPath();
			path = path.equals("/") ? "/index.html" : path;

			String contentType =
					switch (path.substring(path.lastIndexOf("."))) {
						case ".html" -> "text/html";
						case ".css" -> "text/css";
						case ".js" -> "text/javascript";
						default -> "text/plain";
					};

			try {
				byte[] response = Files.readAllBytes(Path.of(deployDirectory, "webserver", path));
				exchange.getResponseHeaders().set("Content-Type", contentType);
				exchange.sendResponseHeaders(200, response.length);
				exchange.getResponseBody().write(response);
			} catch (IOException e) {
				String response = "File not found";
				exchange.sendResponseHeaders(404, response.length());
				exchange.getResponseBody().write(response.getBytes());
			}
			exchange.getResponseBody().close();
		}
	}

	class CommandHandler implements HttpHandler {
		@Override
		public void handle(HttpExchange exchange) throws IOException {
			String query = exchange.getRequestURI().getQuery();
			String response = "Command received";

			if (query != null && query.startsWith("id=")) {
				String commandId = query.substring(3);
				Command cmd = commandMap.get(commandId);
				if (cmd != null) {
					CommandScheduler.getInstance().schedule(cmd);
				}
			}

			exchange.sendResponseHeaders(200, response.length());
			exchange.getResponseBody().write(response.getBytes());
			exchange.getResponseBody().close();
		}
	}

	class StatusHandler implements HttpHandler {
		@Override
		public void handle(HttpExchange exchange) throws IOException {
			String response = "OK";
			exchange.getResponseHeaders().set("Content-Type", "text/plain");
			exchange.getResponseHeaders().set("Access-Control-Allow-Origin", "*");
			exchange.sendResponseHeaders(200, response.length());
			exchange.getResponseBody().write(response.getBytes());
			exchange.getResponseBody().close();
		}
	}
}
