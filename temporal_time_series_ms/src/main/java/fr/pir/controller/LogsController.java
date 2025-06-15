package fr.pir.controller;

import java.io.IOException;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RequestPart;
import org.springframework.web.multipart.MultipartFile;

import fr.phenix333.logger.MyLogger;
import fr.pir.service.LogsService;

@Controller
@RequestMapping("/api/logs")
public class LogsController {

	private static final MyLogger L = MyLogger.create(LogsController.class);

	@Autowired
	private LogsService logsService;

	/**
	 * Save logs in DB from uploaded files for a specific model.
	 * 
	 * @param modelName : String -> The name of the model for which logs are being
	 *                  saved
	 * @param files     : MultipartFile[] -> Array of files containing logs to be
	 *                  saved
	 * 
	 * @return ResponseEntity<String> -> The 'OK' message or the error message
	 */
	@PostMapping(consumes = { "multipart/form-data" })
	public ResponseEntity<String> saveLogs(@RequestParam String modelName,
			@RequestPart("files") MultipartFile[] files) {
		L.function("model name : {}, received files : {}", modelName, files.length);

		try {
			return ResponseEntity.status(HttpStatus.OK).body(this.logsService.saveLogs(modelName, files));
		} catch (IllegalArgumentException e) {
			return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage());
		} catch (IOException e) {
			L.error("IO error while processing file", e);

			return ResponseEntity.status(HttpStatus.BAD_REQUEST).body("Invalid file format or content.");
		} catch (Exception e) {
			L.error("Error saving Logs", e);

			return ResponseEntity.status(HttpStatus.INTERNAL_SERVER_ERROR).body("Oops, something went wrong.");
		}
	}

}
