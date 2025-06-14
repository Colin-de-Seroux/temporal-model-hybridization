package fr.pir.service;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import org.springframework.web.multipart.MultipartFile;

import fr.phenix333.logger.MyLogger;
import fr.pir.model.Action;
import fr.pir.model.Behavior;
import fr.pir.model.Model;
import fr.pir.model.Node;
import fr.pir.repository.ActionRepository;
import fr.pir.repository.ModelRepository;
import jakarta.transaction.Transactional;

@Service
public class LogsService {

	private static final MyLogger L = MyLogger.create(LogsService.class);

	@Autowired
	private ModelRepository modelRepository;

	@Autowired
	private ActionRepository actionRepository;

	/**
	 * Extract logs from a file and return the first element as the key and the
	 * lines as the value.
	 *
	 * @param file : MultipartFile -> The file from which logs are to be extracted
	 *
	 * @return Map.Entry<String, List<String>> -> A map entry where the key is the
	 *         first element of the first line and the value is a list of all lines
	 *         in the file
	 *
	 * @throws IOException
	 */
	private Map.Entry<String, List<String>> extractLogsFromFile(MultipartFile file) throws IOException {
		L.function("file name : {}", file.getOriginalFilename());

		List<String> lines = new BufferedReader(new InputStreamReader(file.getInputStream())).lines()
				.collect(Collectors.toList());

		String firstElement = "";

		if (!lines.isEmpty()) {
			String firstLine = lines.get(0);
			String[] words = firstLine.split("\\s+");

			if (words.length > 0) {
				firstElement = words[words.length - 1];
			}
		}

		return Map.entry(firstElement, lines);
	}

	/**
	 * Extract logs from multiple files and return a map where the key is the first
	 * element of each file and the value is a list of all lines in that file.
	 *
	 * @param files : MultipartFile[] -> An array of files from which logs are to be
	 *              extracted
	 *
	 * @return Map<String, List<String>> -> A map where the key is the first element
	 *         of each file and the value is a list of all lines in that file
	 *
	 * @throws IOException
	 */
	private Map<String, List<String>> extractLogsFromFiles(MultipartFile[] files) throws IOException {
		L.function("received files : {}", files.length);

		Map<String, List<String>> allLogs = new HashMap<>();

		for (MultipartFile file : files) {
			Map.Entry<String, List<String>> logsNode = this.extractLogsFromFile(file);

			allLogs.put(logsNode.getKey(), logsNode.getValue());
		}

		return allLogs;
	}

	/**
	 * Save actions from behaviors into the database.
	 *
	 * @param behaviors : List<Behavior> -> The list of behaviors containing actions
	 */
	private void saveActions(List<Behavior> behaviors) {
		L.function("");

		for (Behavior behavior : behaviors) {
			for (Action action : behavior.getActions()) {
				this.actionRepository.save(action);
			}
		}
	}

	/**
	 * Save logs from uploaded files for a specific model.
	 *
	 * @param modelName : String -> The name of the model for which logs are being
	 *                  saved
	 * @param files     : MultipartFile[] -> Array of files containing logs to be
	 *                  saved
	 *
	 * @return String -> "OK" if logs are saved successfully
	 *
	 * @throws IOException
	 * @throws IllegalArgumentException
	 */
	@Transactional
	public String saveLogs(String modelName, MultipartFile[] files) throws IOException, IllegalArgumentException {
		L.function("model name : {}, received files : {}", modelName, files.length);

		Model model = this.modelRepository.findModelByName(modelName);

		if (model == null) {
			L.error("Model not found: {}", modelName);

			throw new IllegalArgumentException(String.format("Model not found: %s", modelName));
		}

		Map<String, List<String>> allLogs = this.extractLogsFromFiles(files);

		for (Map.Entry<String, List<String>> entry : allLogs.entrySet()) {
			String key = entry.getKey();
			List<String> logs = entry.getValue();

			Node node = model.getNodes().stream().filter(n -> n.getName().equals(key)).findFirst()
					.orElseThrow(() -> new IllegalArgumentException(String.format("Node not found: %s", key)));

			List<Behavior> behaviors = node.getBehaviors().stream()
					.sorted(Comparator.comparing(Behavior::getBehaviorIndex)).collect(Collectors.toList());

			for (String log : logs) {
				String cutLog = log.replaceAll("^\\[.*?\\] \\[.*?\\] \\[.*?\\]: ", "");
				String[] words = cutLog.split("\\s+");
				String identifier = words[0];

				if (identifier.equals("sub") || identifier.equals("pub") || identifier.equals("timer")) {
					cutLog = cutLog.substring(cutLog.indexOf(" ") + 1);

					Behavior behavior = behaviors.get(Integer.parseInt(words[1]));

					List<Action> actions = behavior.getActions().stream()
							.sorted(Comparator.comparing(Action::getActionOrder)).collect(Collectors.toList());

					Action action = actions.get(Integer.parseInt(words[2]));
					action.getExecutionTimes().add(Double.parseDouble(words[3]));
				}
			}

			this.saveActions(behaviors);
		}

		return "OK";
	}

}
