package fr.pir.repository;

import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

import fr.pir.model.Node;

@Repository
public interface NodeRepository extends JpaRepository<Node, Long> {

}
