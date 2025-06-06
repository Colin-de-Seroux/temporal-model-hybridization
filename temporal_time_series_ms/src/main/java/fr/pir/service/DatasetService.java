package fr.pir.service;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVPrinter;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import fr.phenix333.logger.MyLogger;
import fr.pir.model.Action;
import fr.pir.model.Behavior;
import fr.pir.model.Model;
import fr.pir.model.Node;
import fr.pir.repository.ModelRepository;

@Service
public class DatasetService {

    private static final MyLogger L = MyLogger.create(DatasetService.class);

    @Autowired
    private ModelRepository modelRepository;

    /**
     * Generate a dataset from the models in the database and save it as a CSV
     * file.
     *
     * @return Path -> The path to the generated dataset CSV file
     *
     * @throws IOException
     */
    public Path getDataset() throws IOException {
        L.function("");

        List<Model> models = modelRepository.findAll();

        String outputFilePath = "dataset.csv";
        String[] headers = {"ModelName", "NodeName", "BehaviorIndex", "ActionType", "ActionOrder", "Topic", "Value", "ExecutionTime"};

        FileWriter writer = new FileWriter(outputFilePath);
        CSVPrinter csvPrinter = new CSVPrinter(writer, CSVFormat.DEFAULT.withHeader(headers));

        int i = 0;

        for (Model model : models) {
            String[] row = new String[8];

            int j = 0;
            int x = 0;

            Map<String, List<Double>> executionPubMap = new HashMap<>();

            for (Node node : model.getNodes()) {
                for (Behavior behavior : node.getBehaviors()) {
                    for (Action action : behavior.getActions()) {
                        List<Double> executionTimes = action.getExecutionTimes();

                        if (!executionTimes.isEmpty() && executionTimes.size() > x) {
                            x = executionTimes.size();
                        }

                        if (action.getType().equals("pub")) {
                            executionPubMap.putIfAbsent(action.getTopic(), executionTimes);
                        }
                    }
                }
            }

            for (int y = 1; y < x - 1; y++) {
                for (Node node : model.getNodes()) {
                    row[1] = String.format("%s_%d", node.getName(), j++);

                    for (Behavior behavior : node.getBehaviors()) {
                        row[2] = String.valueOf(behavior.getBehaviorIndex());

                        for (Action action : behavior.getActions()) {
                            row[0] = String.format("%s_%d", model.getName(), i);
                            row[3] = action.getType();
                            row[4] = String.valueOf(action.getActionOrder());
                            row[5] = action.getTopic() != null ? String.format("%s_%d", action.getTopic(), i) : "";
                            row[6] = String.valueOf(action.getValue());

                            if (y == 0 || action.getType().equals("pub") || action.getExecutionTimes().isEmpty() || action.getExecutionTimes().size() <= y) {
                                row[7] = "0";
                            } else if (action.getType().equals("timer")) {
                                row[7] = String.format(Locale.US, "%.5f", action.getExecutionTimes().get(y) - action.getExecutionTimes().get(y - 1));
                            } else if (action.getType().equals("sub")) {
                                row[7] = String.format(Locale.US, "%.5f", action.getExecutionTimes().get(y) - executionPubMap.get(action.getTopic()).get(y));
                            }

                            csvPrinter.printRecord((Object[]) row);
                        }
                    }
                }

                i++;
            }
        }

        csvPrinter.flush();
        csvPrinter.close();

        return new File(outputFilePath).toPath().normalize();
    }

}
