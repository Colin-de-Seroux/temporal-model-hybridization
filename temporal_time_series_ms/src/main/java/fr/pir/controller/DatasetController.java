package fr.pir.controller;

import java.io.IOException;
import java.nio.file.Path;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.core.io.Resource;
import org.springframework.core.io.UrlResource;
import org.springframework.http.HttpHeaders;
import org.springframework.http.ResponseEntity;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;

import fr.phenix333.logger.MyLogger;
import fr.pir.service.DatasetService;

@Controller
@RequestMapping("/api/dataset")
public class DatasetController {

    private static final MyLogger L = MyLogger.create(DatasetController.class);

    @Autowired
    private DatasetService datasetService;

    /**
     * Get the dataset file.
     * 
     * @return ResponseEntity<Resource> | ResponseEntity<String> -> The dataset file or the error message
     */
    @GetMapping
    public ResponseEntity<?> getDataset() {
        L.function("");

        try {
            Path dataset = this.datasetService.getDataset();
            Resource resource = new UrlResource(dataset.toUri());

            return ResponseEntity.status(HttpStatus.OK)
                    .contentType(MediaType.APPLICATION_OCTET_STREAM)
                    .header(HttpHeaders.CONTENT_DISPOSITION, "attachment; filename=\"" + resource.getFilename() + "\"")
                    .body(resource);
        } catch (IllegalArgumentException e) {
            return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage());
        } catch (IOException e) {
            L.error("IO error while processing dataset", e);

            return ResponseEntity.status(HttpStatus.BAD_REQUEST).body(String.format("IO error while processing dataset: ", e.getMessage()));
        } catch (Exception e) {
            L.error("Error creating dataset", e);

            return ResponseEntity.status(HttpStatus.INTERNAL_SERVER_ERROR)
                    .body(String.format("Error creating dataset: ", e.getMessage()));
        }
    }

}
