package fr.pir.controller;

import java.io.IOException;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.dao.DataIntegrityViolationException;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestPart;
import org.springframework.web.multipart.MultipartFile;

import fr.phenix333.logger.MyLogger;
import fr.pir.service.JsonGraphService;

import org.springframework.web.bind.annotation.PostMapping;

@Controller
@RequestMapping("/api/graph")
public class JsonGraphController {

    private static final MyLogger L = MyLogger.create(JsonGraphController.class);

    @Autowired
    private JsonGraphService jsonGraphService;

    @PostMapping(consumes = {"multipart/form-data"})
    public ResponseEntity<?> saveJsonGraph(@RequestPart("file") MultipartFile file) {
        L.info("received file : {}", file.getOriginalFilename());

        try {
            return ResponseEntity.status(HttpStatus.OK).body(this.jsonGraphService.saveJsonGraph(file));
        } catch (DataIntegrityViolationException e) {
            L.error("Data integrity violation while saving JSON graph", e);

            return ResponseEntity.status(HttpStatus.CONFLICT).body("Duplicate entry.");
        } catch (IOException e) {
            L.error("IO error while processing file", e);

            return ResponseEntity.status(HttpStatus.BAD_REQUEST).body("Invalid file format or content.");
        } catch (Exception e) {
            L.error("Error saving JSON graph", e);

            return ResponseEntity.status(HttpStatus.INTERNAL_SERVER_ERROR).body("Oops, something went wrong.");
        }
    }

}
