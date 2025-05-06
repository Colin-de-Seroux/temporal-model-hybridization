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

	@GetMapping(value = { "/", "/hello", "/api", "/api/hello" })
	public ResponseEntity<String> helloWorld() {
		L.function("");

		return ResponseEntity.status(HttpStatus.OK).body("Hello World!");
	}

}