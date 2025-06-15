package fr.pir.model;

import java.util.HashSet;
import java.util.Set;

import com.fasterxml.jackson.annotation.JsonIgnore;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.FetchType;
import jakarta.persistence.GeneratedValue;
import jakarta.persistence.GenerationType;
import jakarta.persistence.Id;
import jakarta.persistence.Index;
import jakarta.persistence.ManyToOne;
import jakarta.persistence.OneToMany;
import jakarta.persistence.Table;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@Getter
@NoArgsConstructor
@Setter
@Entity
@Table(name = "nodes", indexes = { @Index(name = "idx_node_name", columnList = "name") })
public class Node {

	@Id
	@GeneratedValue(strategy = GenerationType.AUTO)
	private Long id;

	@Column(nullable = false)
	private String name;

	/**
	 * The estimated time of the node in milliseconds by the user.
	 */
	@Column(nullable = false)
	private double expectedExecTime;

	@OneToMany(mappedBy = "node", fetch = FetchType.LAZY)
	private Set<Behavior> behaviors = new HashSet<>();

	@JsonIgnore
	@ManyToOne
	private Model model;

}
