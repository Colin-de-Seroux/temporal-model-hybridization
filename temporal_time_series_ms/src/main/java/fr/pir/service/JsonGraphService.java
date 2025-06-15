package fr.pir.service;

import java.io.IOException;
import java.nio.charset.StandardCharsets;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import org.springframework.web.multipart.MultipartFile;

import com.google.gson.Gson;

import fr.phenix333.logger.MyLogger;
import fr.pir.model.Action;
import fr.pir.model.Behavior;
import fr.pir.model.Model;
import fr.pir.model.Node;
import fr.pir.repository.ActionRepository;
import fr.pir.repository.BehaviorRepository;
import fr.pir.repository.ModelRepository;
import fr.pir.repository.NodeRepository;
import jakarta.transaction.Transactional;

@Service
public class JsonGraphService {

	private static final MyLogger L = MyLogger.create(JsonGraphService.class);

	@Autowired
	private ModelRepository modelRepository;

	@Autowired
	private NodeRepository nodeRepository;

	@Autowired
	private BehaviorRepository behaviorRepository;

	@Autowired
	private ActionRepository actionRepository;

	/**
	 * Create an Action with the given parameters and save it to the database.
	 *
	 * @param action   : Action -> The action to be created
	 * @param behavior : Behavior -> The behavior to which the action belongs
	 * @param order    : int -> The order of the action within the behavior
	 *
	 * @return Action -> The saved action with its ID set
	 */
	private Action createAction(Action action, Behavior behavior, int order) {
		L.function("action order : {}", order);

		action.setBehavior(behavior);
		action.setActionOrder(order);

		return this.actionRepository.save(action);
	}

	/**
	 * Create a Behavior with the given parameters and save it to the database.
	 *
	 * @param behavior : Behavior -> The behavior to be created
	 * @param node     : Node -> The node to which the behavior belongs
	 * @param index    : int -> The index of the behavior within the node
	 *
	 * @return Behavior -> The saved behavior with its ID set
	 */
	private Behavior createBehavior(Behavior behavior, Node node, int index) {
		L.function("behavior index : {}", index);

		behavior.setNode(node);
		behavior.setBehaviorIndex(index);

		return this.behaviorRepository.save(behavior);
	}

	/**
	 * Create a Node with the given parameters and save it to the database.
	 *
	 * @param node  : Node -> The node to be created
	 * @param model : Model -> The model to which the node belongs
	 *
	 * @return Node -> The saved node with its ID set
	 */
	private Node createNode(Node node, Model model) {
		L.function("node name : {}", node.getName());

		node.setModel(model);

		return this.nodeRepository.save(node);
	}

	/**
	 * Create a Model with the given name and save it to the database.
	 *
	 * @param name : String -> The name of the model to be created
	 *
	 * @return Model -> The saved model with its ID set
	 */
	public Model createModel(String name) {
		L.function("model name : {}", name);

		Model model = new Model(name);

		return this.modelRepository.save(model);
	}

	/**
	 * Save a JSON graph from a file and create the corresponding Model, Nodes,
	 * Behaviors, and Actions in the database.
	 *
	 * @param file : MultipartFile -> The file containing the JSON graph to be saved
	 *
	 * @return Model -> The saved model with its ID set, containing the nodes,
	 *         behaviors, and actions
	 *
	 * @throws IOException
	 */
	@Transactional
	public Model saveJsonGraph(MultipartFile file) throws IOException {
		L.function("file name : {}", file.getOriginalFilename());

		String content = new String(file.getBytes(), StandardCharsets.UTF_8);

		Gson gson = new Gson();
		Model gsonModel = gson.fromJson(content, Model.class);

		Model model = this.createModel(gsonModel.getName());

		for (Node node : gsonModel.getNodes()) {
			this.createNode(node, model);

			int index = 0;

			for (Behavior behavior : node.getBehaviors()) {
				this.createBehavior(behavior, node, index++);

				int order = 0;

				for (Action action : behavior.getActions()) {
					this.createAction(action, behavior, order++);
				}
			}
		}

		gsonModel.setId(model.getId());

		return gsonModel;
	}

}
