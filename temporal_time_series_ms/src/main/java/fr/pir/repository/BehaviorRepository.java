package fr.pir.repository;

import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

import fr.pir.model.Behavior;

@Repository
public interface BehaviorRepository extends JpaRepository<Behavior, Long> {

}
