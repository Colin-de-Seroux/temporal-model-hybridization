package fr.pir.repository;

import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

import fr.pir.model.Action;

@Repository
public interface ActionRepository extends JpaRepository<Action, Long> {

}
