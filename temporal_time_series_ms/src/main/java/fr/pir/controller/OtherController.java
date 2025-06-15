package fr.pir.controller;

import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;

import fr.phenix333.logger.MyLogger;

@Controller
@RequestMapping
public class OtherController {

	private static final MyLogger L = MyLogger.create(OtherController.class);

	/**
	 * Endpoint to return a simple "Hello World!" message.
	 *
	 * @return ResponseEntity<String> -> The 'Hello World!' message with HTTP status
	 *         OK
	 */
	@GetMapping({ "/hello", "/api/hello" })
	public ResponseEntity<String> helloWorld() {
		L.function("");

		return ResponseEntity.status(HttpStatus.OK).body("Hello World!");
	}

	/**
	 * Endpoint to serve the Swagger UI HTML file.
	 *
	 * @return String -> Redirects to the static swagger.html file
	 */
	@GetMapping({ "/", "/api", "/swagger", "/public" })
	public String redirectToSwagger() {
		return "redirect:/swagger.html";
	}

}
